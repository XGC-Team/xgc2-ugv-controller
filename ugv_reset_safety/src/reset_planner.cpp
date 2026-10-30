#include <ugv_reset_safety/reset_path.h>
#include <ugv_reset_safety/reset_planner.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ugv_reset_safety {
namespace {

namespace rc = ugv_reset_client;
using rc::GrantState;
using rc::Phase;
using rc::Pose;
using rc::Reason;
using rc::Route;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kChunk = 8;            // samples per broadphase box
constexpr int kBehind = 6;           // samples kept reserved behind a mover's cursor
constexpr double kCycleReach = 2.0;  // m of free prefix that justify breaking a cycle of precedence
constexpr int kWindow = 160;         // cursor search window [samples]

// ---------------------------------------------------------------------------------------------
// Geometry: convex polygons, separating-axis clearance.

struct Aabb {
    double minx{kInf}, maxx{-kInf}, miny{kInf}, maxy{-kInf};
    void add(double x, double y) {
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    }
    void add(const Aabb& o) {
        minx = std::min(minx, o.minx);
        maxx = std::max(maxx, o.maxx);
        miny = std::min(miny, o.miny);
        maxy = std::max(maxy, o.maxy);
    }
};

// True if the boxes are at least m apart along one axis, hence so are their contents.
inline bool separated(const Aabb& a, const Aabb& b, double m) {
    return a.minx - b.maxx >= m || b.minx - a.maxx >= m || a.miny - b.maxy >= m ||
           b.miny - a.maxy >= m;
}

struct View {
    const double* x;
    const double* y;
    int n;
    bool rect;  // the first two edges are the two axes of a rectangle
};

double separation(const View& a, const View& b, double early) {
    double best = -kInf;
    for (int side = 0; side < 2; ++side) {
        const View& p = side == 0 ? a : b;
        const int axes = p.rect ? 2 : p.n;
        for (int e = 0; e < axes; ++e) {
            const int e2 = (e + 1) % p.n;
            const double tx = p.x[e2] - p.x[e];
            const double ty = p.y[e2] - p.y[e];
            const double length = std::hypot(tx, ty);
            if (length < 1e-12) {
                continue;
            }
            const double nx = -ty / length;
            const double ny = tx / length;
            double amin = kInf, amax = -kInf, bmin = kInf, bmax = -kInf;
            for (int i = 0; i < a.n; ++i) {
                const double d = nx * a.x[i] + ny * a.y[i];
                amin = std::min(amin, d);
                amax = std::max(amax, d);
            }
            for (int i = 0; i < b.n; ++i) {
                const double d = nx * b.x[i] + ny * b.y[i];
                bmin = std::min(bmin, d);
                bmax = std::max(bmax, d);
            }
            const double gap = std::max(bmin - amax, amin - bmax);
            if (gap > best) {
                best = gap;
                if (best >= early) {
                    return best;
                }
            }
        }
    }
    return best;
}

double segmentDistance(double px, double py, double ax, double ay, double bx, double by) {
    const double ex = bx - ax, ey = by - ay;
    const double l2 = ex * ex + ey * ey;
    const double t = l2 > 0.0 ? std::clamp(((px - ax) * ex + (py - ay) * ey) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(px - ax - t * ex, py - ay - t * ey);
}

// Distance between two separated convex polygons.
double exactDistance(const View& a, const View& b) {
    double best = kInf;
    for (int i = 0; i < a.n; ++i) {
        for (int e = 0; e < b.n; ++e) {
            const int e2 = (e + 1) % b.n;
            best =
                std::min(best, segmentDistance(a.x[i], a.y[i], b.x[e], b.y[e], b.x[e2], b.y[e2]));
        }
    }
    for (int i = 0; i < b.n; ++i) {
        for (int e = 0; e < a.n; ++e) {
            const int e2 = (e + 1) % a.n;
            best =
                std::min(best, segmentDistance(b.x[i], b.y[i], a.x[e], a.y[e], a.x[e2], a.y[e2]));
        }
    }
    return best;
}

// The exact clearance test: the polygons are at least `margin` apart (Euclidean). A separating
// axis with a gap of `margin` decides at once; a smaller positive gap needs the exact distance
// (two corners facing each other diagonally can be farther apart than any axis gap).
bool clearAtLeast(const View& a, const View& b, double margin) {
    const double gap = separation(a, b, margin);
    if (gap >= margin) {
        return true;
    }
    if (gap <= 0.0) {
        return false;
    }
    return exactDistance(a, b) >= margin;
}

double segmentClosest(double px, double py, double ax, double ay, double bx, double by,
                      double* t, double* qx, double* qy) {
    const double ex = bx - ax, ey = by - ay;
    const double l2 = ex * ex + ey * ey;
    *t = l2 > 0.0 ? std::clamp(((px - ax) * ex + (py - ay) * ey) / l2, 0.0, 1.0) : 0.0;
    *qx = ax + *t * ex;
    *qy = ay + *t * ey;
    return std::hypot(px - *qx, py - *qy);
}

// The closest feature pair of two separated convex polygons: the exact distance, the unit
// direction peer -> self at that pair, and whether the minimum is achieved by a peer vertex
// against the INTERIOR of a self edge (the tangent-lock case, where the separating halfspace
// must live in self's body frame instead of the world frame).
struct ClosestPair {
    double d{kInf};
    double nx{0.0}, ny{0.0};
    bool peer_v_to_self_e{false};
};

ClosestPair closestPair(const View& self_v, const View& peer_v) {
    ClosestPair out;
    for (int i = 0; i < self_v.n; ++i) {
        for (int e = 0; e < peer_v.n; ++e) {
            const int e2 = (e + 1) % peer_v.n;
            double t, qx, qy;
            const double d = segmentClosest(self_v.x[i], self_v.y[i], peer_v.x[e], peer_v.y[e],
                                            peer_v.x[e2], peer_v.y[e2], &t, &qx, &qy);
            if (d < out.d) {
                out.d = d;
                out.nx = self_v.x[i] - qx;
                out.ny = self_v.y[i] - qy;
                out.peer_v_to_self_e = false;
            }
        }
    }
    for (int j = 0; j < peer_v.n; ++j) {
        for (int e = 0; e < self_v.n; ++e) {
            const int e2 = (e + 1) % self_v.n;
            double t, qx, qy;
            const double d = segmentClosest(peer_v.x[j], peer_v.y[j], self_v.x[e], self_v.y[e],
                                            self_v.x[e2], self_v.y[e2], &t, &qx, &qy);
            if (d < out.d) {
                out.d = d;
                out.nx = qx - peer_v.x[j];
                out.ny = qy - peer_v.y[j];
                out.peer_v_to_self_e = t > 1e-6 && t < 1.0 - 1e-6;
            }
        }
    }
    const double norm = std::hypot(out.nx, out.ny);
    if (norm > 1e-12) {
        out.nx /= norm;
        out.ny /= norm;
    }
    return out;
}

// A footprint in the body frame: the declared rectangle, or for an omnidirectional vehicle the
// 16-gon that circumscribes the disc of the rectangle's circumradius (any heading is covered).
struct Shape {
    bool rect{true};
    double bx[16]{}, by[16]{};
    double radius{0.0};  // farthest point from the body origin
};

Shape makeShape(const rc::VehicleSpec& spec) {
    Shape shape;
    const rc::Footprint& f = spec.footprint;
    const double cx[4] = {-f.half_length, f.half_length, f.half_length, -f.half_length};
    const double cy[4] = {-f.half_width, -f.half_width, f.half_width, f.half_width};
    for (int k = 0; k < 4; ++k) {
        shape.bx[k] = cx[k] + f.offset_x;
        shape.by[k] = cy[k] + f.offset_y;
        shape.radius = std::max(shape.radius, std::hypot(shape.bx[k], shape.by[k]));
    }
    if (spec.model == rc::Model::Omni) {
        shape.rect = false;
        const double r = shape.radius / std::cos(rc::kPi / 16.0);
        for (int k = 0; k < 16; ++k) {
            shape.bx[k] = r * std::cos(2.0 * rc::kPi * k / 16.0);
            shape.by[k] = r * std::sin(2.0 * rc::kPi * k / 16.0);
        }
    }
    return shape;
}

struct Verts {
    double x[16], y[16];
    int n;
    bool rect;
    View view() const {
        return {x, y, n, rect};
    }
};

void place(const Shape& shape, double px, double py, double yaw, Verts* out) {
    out->n = shape.rect ? 4 : 16;
    out->rect = shape.rect;
    if (!shape.rect) {
        for (int k = 0; k < 16; ++k) {
            out->x[k] = px + shape.bx[k];
            out->y[k] = py + shape.by[k];
        }
        return;
    }
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (int k = 0; k < 4; ++k) {
        out->x[k] = px + c * shape.bx[k] - s * shape.by[k];
        out->y[k] = py + s * shape.bx[k] + c * shape.by[k];
    }
}

Aabb boxOf(const Verts& v) {
    Aabb box;
    for (int k = 0; k < v.n; ++k) {
        box.add(v.x[k], v.y[k]);
    }
    return box;
}

struct Sample {
    double x, y, yaw, s;
    Aabb box;
};

// The sweep of one rotation-centre offset: the footprint at every sample of the route.
struct SampleSet {
    std::vector<Sample> pts;
    std::vector<Aabb> chunks;
    double ell{0.0};  // the variant's rotation-centre offset (Route::ell)
};

SampleSet makeSet(const Route& route, const Shape& shape, bool unicycle,
                  const PlannerConfig& config) {
    SampleSet set;
    const auto raw = route.sample(config.step, config.step_yaw, unicycle);
    set.pts.reserve(raw.size());
    for (const auto& r : raw) {
        Verts v;
        place(shape, r.pose.x, r.pose.y, r.pose.yaw, &v);
        set.pts.push_back({r.pose.x, r.pose.y, r.pose.yaw, r.s, boxOf(v)});
    }
    for (size_t i = 0; i < set.pts.size(); i += kChunk) {
        Aabb box;
        for (size_t k = i; k < std::min(set.pts.size(), i + kChunk); ++k) {
            box.add(set.pts[k].box);
        }
        set.chunks.push_back(box);
    }
    set.ell = route.ell();
    return set;
}

constexpr size_t kNoEscape = std::numeric_limits<size_t>::max();

// Admission-owned escape certificate for one parked peer the route inherits an
// inside-margin gap from (0 < d0 < pair margin): the route may leave the
// overlap, but on the way out it may never close below d0 along a separating
// halfspace frozen at admission. Once a full-margin separation is achieved the
// certificate retires and the ordinary pair margin applies; before that every
// swept step must keep the halfspace, in continuous time, not just at samples.
struct EscapeCert {
    std::string peer_id;
    Pose peer_pose;
    Verts peer_verts;  // the peer footprint at peer_pose, frozen with the cert
    // false: n = (nx, ny) is the world-frame separating normal, peer -> self.
    // true:  n = (ux, uy) is the normal in self's body frame at the admission
    //        yaw, self -> peer; self's edge is then stationary and the peer's
    //        vertices swing around it (resolves the s=0 tangent lock).
    bool body_fixed{false};
    double nx{0.0}, ny{0.0};
    double d0{0.0};    // the inherited gap the certificate preserves
    double rhs{0.0};   // n . self_vertex_min >= rhs  (n . peer_vertex_min >= rhs in body frame)
    // Per sweep variant: the first route sample whose footprint clears the
    // peer by the full pair margin (kNoEscape = never). Cached at admission so
    // the escape state is a pure function of the frozen route index - no
    // per-tick replay of the driven prefix, and no d0 allowance is regained
    // when a far peer re-enters the broadphase.
    std::vector<size_t> escaped_at;
};

// Exact continuous-time check of one swept step against an escape certificate.
// pts[i-1] -> pts[i] is one primitive: a pure rotation in place when
// pts[i].s == pts[i-1].s (vertex projections are sinusoids in yaw; the exact
// interval minimum sits at an endpoint or the stationary angle atan2(-B,-A)),
// or a fixed-yaw translation otherwise (projections are affine in s, so the
// endpoint check already bounds the interval).
bool escapeStepClear(const Shape& self_shape, const SampleSet& set, size_t i,
                     const EscapeCert& cert) {
    constexpr double kTol = 1e-9;
    const Sample& q = set.pts[i];
    if (!cert.body_fixed) {
        Verts va;
        place(self_shape, q.x, q.y, q.yaw, &va);
        double m = kInf;
        for (int k = 0; k < va.n; ++k) {
            m = std::min(m, cert.nx * va.x[k] + cert.ny * va.y[k]);
        }
        if (m < cert.rhs - kTol) {
            return false;
        }
    } else {
        const double c = std::cos(q.yaw), s = std::sin(q.yaw);
        double m = kInf;
        for (int k = 0; k < cert.peer_verts.n; ++k) {
            const double dx = cert.peer_verts.x[k] - q.x, dy = cert.peer_verts.y[k] - q.y;
            const double bx = dx * c + dy * s;   // R(-yaw) * (peer_vertex - origin)
            const double by = -dx * s + dy * c;
            m = std::min(m, cert.nx * bx + cert.ny * by);
        }
        if (m < cert.rhs - kTol) {
            return false;
        }
    }
    if (i == 0) {
        return true;
    }
    const Sample& prev = set.pts[i - 1];
    if (q.s != prev.s) {
        return true;  // fixed-yaw translation: affine in s, endpoints bound it
    }
    const double dpsi = rc::wrapAngle(q.yaw - prev.yaw);
    if (dpsi == 0.0) {
        return true;
    }
    const double lo = std::min(0.0, dpsi), hi = std::max(0.0, dpsi);
    // The rotation pivot: q sits on the circle about the route vertex,
    // c = q.origin + ell * (cos yaw, sin yaw).
    const double cx = q.x + set.ell * std::cos(q.yaw);
    const double cy = q.y + set.ell * std::sin(q.yaw);
    auto interior_clear = [&](double A, double B, double C) {
        const double psi = std::atan2(-B, -A);  // the global-min critical angle
        const double off = rc::wrapAngle(psi - prev.yaw);
        return off < lo || off > hi ||
               C + A * std::cos(psi) + B * std::sin(psi) >= cert.rhs - kTol;
    };
    if (!cert.body_fixed) {
        // World vertex of body corner k: c + R(psi) (bx-ell, by); its
        // n-projection is C + A cos psi + B sin psi.
        const int nv = self_shape.rect ? 4 : 16;
        for (int k = 0; k < nv; ++k) {
            const double bx = self_shape.bx[k] - set.ell, by = self_shape.by[k];
            const double A = cert.nx * bx + cert.ny * by;
            const double B = cert.ny * bx - cert.nx * by;
            if (std::hypot(A, B) < 1e-12) {
                continue;  // vertex pinned at the pivot: constant projection
            }
            if (!interior_clear(A, B, cert.nx * cx + cert.ny * cy)) {
                return false;
            }
        }
    } else {
        // Peer vertex y_m mapped into self's body frame:
        // (ell, 0) + R(-psi) (y_m - c); its u-projection is C' + A' cos psi +
        // B' sin psi.
        for (int k = 0; k < cert.peer_verts.n; ++k) {
            const double dx = cert.peer_verts.x[k] - cx, dy = cert.peer_verts.y[k] - cy;
            const double A = cert.nx * dx + cert.ny * dy;
            const double B = cert.nx * dy - cert.ny * dx;
            if (std::hypot(A, B) >= 1e-12 &&
                !interior_clear(A, B, cert.nx * set.ell)) {
                return false;
            }
        }
    }
    return true;
}

struct StaticPoly {
    std::vector<double> x, y;
    Aabb box;
    std::string id;
};

bool outsideFence(const Verts& v, const Fence& fence, double margin) {
    if (!fence.enabled) {
        return false;
    }
    for (int k = 0; k < v.n; ++k) {
        if (v.x[k] < fence.xmin + margin || v.x[k] > fence.xmax - margin ||
            v.y[k] < fence.ymin + margin || v.y[k] > fence.ymax - margin) {
            return true;
        }
    }
    return false;
}

double wrap(double a) {
    return rc::wrapAngle(a);
}

// ---------------------------------------------------------------------------------------------
// Bodies.

struct Plan {
    uint32_t session{0}, request_generation{0};
    Route route;  // the planned rotation-centre offset
    std::vector<SampleSet> sets;
    std::vector<size_t> cursor, hi;
    double s_limit{0.0};
    double total{0.0};
    uint32_t grant_id{0};
    bool final_move{true};
    Pose goal;
    std::vector<rc::Point> via;
    // Escape certificates owned by THIS route admission: one per parked peer
    // the route's start pose overlapped inside the pair margin. They bind the
    // admitted route to its frozen start geometry only - a peer that moved
    // after admission, a new peer, and any re-admitted route get fresh
    // certificates or the full margin.
    std::vector<EscapeCert> escape_certs;
};

struct Body {
    double delta{0.0};  // sampling error of this body's sweeps
    double ell_hat{0.0}, ell_spread{-1.0};
    double since{0.0};  // the session entered the planner
    double occupied_since{-1.0};
    double no_route_since{-1.0};
    double last_limit{0.0};
    Pose pose, target, progress_pose;
    // The pose this body's own last check discharged: drift from an
    // unverified report is measured against this anchor, never against the
    // previous observation, so sub-epsilon drift still accumulates past it.
    // The anchor moves only inside this body's staleness check - an
    // unrelated verification never erases a pending change.
    Pose proof_pose;
    bool proof_pose_set{false};
    std::vector<std::string> last_blockers, blockers;
    std::string id;
    rc::VehicleSpec spec;
    Shape shape;
    Plan plan;
    uint32_t session{0};
    uint32_t request_generation{0}, barrier_id{0}, budget_session{0};
    uint32_t last_grant{0}, grants{0};
    bool awaiting_stop{false}, certified{false}, plan_retiring{false}, go_received{false};
    Phase phase{Phase::Idle};
    bool uncontrolled{false};
    bool participant{false};
    bool rejected{false};  // the planner ended this session
    bool plan_active{false};
    bool emitted{false};  // whether the last decision has been handed out
    GrantState last_state{GrantState::Hold};
    Reason last_reason{Reason::None};
    Reason hold_reason{Reason::None};  // what the vehicle is told while it waits
};

struct Violation {
    bool hard{false};
    bool world{false};  // the blocker is a static or the fence
    double s{kInf};
    std::vector<std::string> blockers;
};

struct Candidate {
    bool ok{false};
    bool route_failed{false};  // the visibility graph found no route even among the statics
    Plan plan;
    std::string why;
    std::vector<std::string> blockers;
};

}  // namespace

// ---------------------------------------------------------------------------------------------

struct ResetPlanner::Impl {
    PlannerConfig cfg;
    std::vector<StaticPoly> statics;
    std::vector<ConvexObstacle> static_list;  // the same, for the visibility graph
    Fence fence;
    bool world_changed{false};
    std::map<std::string, Body> bodies;
    PlannerStats stats;
    std::vector<PlannerDecision> out;
    double now{0.0};
    double last_replan{-1e9};
    double last_extension{-1e9};
    double fleet_progress{0.0};
    bool dirty{true};
    uint32_t next_grant{1};

    explicit Impl(const PlannerConfig& config) : cfg(config) {}

    uint32_t allocate() {
        if (next_grant == 0 || next_grant == std::numeric_limits<uint32_t>::max()) {
            throw std::overflow_error("Reset decision allocator exhausted");
        }
        return next_grant++;
    }

    // ----------------------------------------------------------------- clearance helpers

    Sample poseSample(const Body& body, const Pose& p) const {
        Verts v;
        place(body.shape, p.x, p.y, p.yaw, &v);
        return {p.x, p.y, p.yaw, 0.0, boxOf(v)};
    }

    static bool sampleClear(const Body& a, const Sample& sa, const Body& b, const Sample& sb,
                            double margin) {
        if (std::hypot(sa.x - sb.x, sa.y - sb.y) > a.shape.radius + b.shape.radius + margin) {
            return true;
        }
        if (separated(sa.box, sb.box, margin)) {
            return true;
        }
        Verts va, vb;
        place(a.shape, sa.x, sa.y, sa.yaw, &va);
        place(b.shape, sb.x, sb.y, sb.yaw, &vb);
        return clearAtLeast(va.view(), vb.view(), margin);
    }

    static bool samePose(const Pose& a, const Pose& b) {
        return std::abs(a.x - b.x) < 1e-6 && std::abs(a.y - b.y) < 1e-6 &&
               std::abs(wrap(a.yaw - b.yaw)) < 1e-6;
    }

    // The admission certificate a parked peer carries, if any: peer id AND the
    // pose the certificate froze must both match. A peer that parked or moved
    // after this route's admission has no certificate and clears by the full
    // pair margin.
    static const EscapeCert* matchCert(const Body& peer,
                                       const std::vector<EscapeCert>& certs) {
        for (const EscapeCert& cert : certs) {
            if (cert.peer_id == peer.id && samePose(cert.peer_pose, peer.pose)) {
                return &cert;
            }
        }
        return nullptr;
    }

    bool poseClearOfStatics(const Body& body, const Pose& p, double margin,
                            std::string* blocker = nullptr) const {
        Verts v;
        place(body.shape, p.x, p.y, p.yaw, &v);
        if (outsideFence(v, fence, cfg.fence_margin)) {
            if (blocker != nullptr) {
                *blocker = "fence";
            }
            return false;
        }
        const Aabb box = boxOf(v);
        for (const StaticPoly& s : statics) {
            if (separated(box, s.box, margin)) {
                continue;
            }
            const View sv{s.x.data(), s.y.data(), static_cast<int>(s.x.size()), false};
            if (!clearAtLeast(v.view(), sv, margin)) {
                if (blocker != nullptr) {
                    *blocker = s.id;
                }
                return false;
            }
        }
        return true;
    }

    // ----------------------------------------------------------------- the sweep verifier

    // First violation of the sweeps `sets` of `self`, from the per-variant start index `from`.
    // Hard: a static, the fence or a parked body (it does not go away by waiting for a mover).
    // Soft: the pose or the remaining reserved sweep of a mover (it shrinks as the mover advances).
    // A parked peer carrying an admission escape certificate is checked against that
    // certificate until the frozen escape index; the full pair margin applies after.
    Violation firstViolation(const Body& self, const std::vector<const SampleSet*>& sets,
                             const std::vector<size_t>& from,
                             const std::vector<EscapeCert>& certs) const {
        Violation result;
        const double m_static = cfg.static_margin + self.delta;
        const double m_fence = cfg.fence_margin + self.delta;
        const double m_parked = cfg.pair_margin + self.delta;
        // Chunk of a mover's reservation that may touch the candidate chunk.
        struct MoverChunk {
            const Body* body;
            size_t variant, chunk;
        };
        std::vector<size_t> rel_statics;
        std::vector<const Body*> rel_parked;
        std::vector<MoverChunk> rel_movers;
        std::vector<Sample> rel_mover_now;
        std::vector<const Body*> now_bodies;
        for (size_t v = 0; v < sets.size(); ++v) {
            const SampleSet& set = *sets[v];
            bool soft_found = false;
            for (size_t c = from[v] / kChunk; c < set.chunks.size(); ++c) {
                const Aabb& cb = set.chunks[c];
                const size_t first = std::max(c * kChunk, from[v]);
                const size_t last = std::min(set.pts.size(), (c + 1) * kChunk);
                bool inside_fence =
                    !fence.enabled ||
                    (cb.minx >= fence.xmin + m_fence && cb.maxx <= fence.xmax - m_fence &&
                     cb.miny >= fence.ymin + m_fence && cb.maxy <= fence.ymax - m_fence);
                rel_statics.clear();
                rel_parked.clear();
                rel_movers.clear();
                now_bodies.clear();
                rel_mover_now.clear();
                for (size_t i = 0; i < statics.size(); ++i) {
                    if (!separated(cb, statics[i].box, m_static)) {
                        rel_statics.push_back(i);
                    }
                }
                for (const auto& entry : bodies) {
                    const Body& other = entry.second;
                    if (&other == &self) {
                        continue;
                    }
                    if (!other.plan_active) {
                        const Sample at = poseSample(other, other.pose);
                        // A separated chunk holds the full margin at every
                        // sample and therefore lies entirely past the escape
                        // index; only the chunk still containing index <=
                        // escaped_at can carry an unverified escape step into
                        // the first separated sample.
                        const EscapeCert* cert = matchCert(other, certs);
                        if (!separated(cb, at.box, m_parked) ||
                            (cert != nullptr && v < cert->escaped_at.size() &&
                             first <= cert->escaped_at[v])) {
                            rel_parked.push_back(&other);
                        }
                        continue;
                    }
                    if (soft_found && !other.plan_retiring) {
                        continue;
                    }
                    const double m_mover = cfg.pair_margin + self.delta + other.delta;
                    const Sample at = poseSample(other, other.pose);
                    if (!separated(cb, at.box, m_mover)) {
                        now_bodies.push_back(&other);
                        rel_mover_now.push_back(at);
                    }
                    for (size_t w = 0; w < other.plan.sets.size(); ++w) {
                        const SampleSet& reserved = other.plan.sets[w];
                        const size_t lo =
                            other.plan.cursor[w] > kBehind ? other.plan.cursor[w] - kBehind : 0;
                        if (other.plan.hi[w] <= lo) {
                            continue;
                        }
                        for (size_t k = lo / kChunk; k * kChunk < other.plan.hi[w]; ++k) {
                            if (!separated(cb, reserved.chunks[k], m_mover)) {
                                rel_movers.push_back({&other, w, k});
                            }
                        }
                    }
                }
                if (inside_fence && rel_statics.empty() && rel_parked.empty() &&
                    rel_movers.empty() && now_bodies.empty()) {
                    continue;
                }
                for (size_t i = first; i < last; ++i) {
                    const Sample& q = set.pts[i];
                    if (!inside_fence || !rel_statics.empty()) {
                        Verts v;
                        place(self.shape, q.x, q.y, q.yaw, &v);
                        if (outsideFence(v, fence, m_fence)) {
                            result.hard = true;
                            result.world = true;
                            result.s = q.s;
                            result.blockers = {"fence"};
                            return result;
                        }
                        for (size_t k : rel_statics) {
                            const StaticPoly& st = statics[k];
                            if (separated(q.box, st.box, m_static)) {
                                continue;
                            }
                            const View sv{st.x.data(), st.y.data(), static_cast<int>(st.x.size()),
                                          false};
                            if (!clearAtLeast(v.view(), sv, m_static)) {
                                result.hard = true;
                                result.world = true;
                                result.s = q.s;
                                result.blockers = {st.id};
                                return result;
                            }
                        }
                    }
                    for (const Body* p : rel_parked) {
                        const EscapeCert* cert = matchCert(*p, certs);
                        // Escape state is a pure function of the frozen route
                        // index: at or before the admission-cached escape index
                        // the step into q must keep the certificate's
                        // halfspace; past it the ordinary pair margin applies.
                        // The index is evaluated per variant and survives both
                        // the broadphase skips of far chunks and a from[v] > 0
                        // extension cursor - no prefix replay, no regained
                        // allowance.
                        const bool escaped = cert == nullptr ||
                                             (v < cert->escaped_at.size() &&
                                              i > cert->escaped_at[v]);
                        const bool clear =
                            escaped ? sampleClear(self, q, *p, poseSample(*p, p->pose), m_parked)
                                    : escapeStepClear(self.shape, set, i, *cert);
                        if (!clear) {
                            result.hard = true;
                            result.s = q.s;
                            result.blockers = {p->id};
                            return result;
                        }
                    }
                    // A retirement barrier freezes the highest issued prefix and rear cursor.
                    // It cannot be treated as a mover whose reservation will shrink by waiting.
                    // Check its ENTIRE existing per-variant retained sample interval before the
                    // soft-conflict shortcut. No new hull/polygon or pose-only substitute is used.
                    auto reservedConflict = [&](bool retiring, std::string& who) {
                        for (size_t k = 0; k < now_bodies.size(); ++k) {
                            const Body& other = *now_bodies[k];
                            if (other.plan_retiring != retiring) {
                                continue;
                            }
                            const double m = cfg.pair_margin + self.delta + other.delta;
                            if (!sampleClear(self, q, other, rel_mover_now[k], m)) {
                                who = other.id;
                                return true;
                            }
                        }
                        for (const MoverChunk& mc : rel_movers) {
                            const Body& other = *mc.body;
                            if (other.plan_retiring != retiring) {
                                continue;
                            }
                            const double m = cfg.pair_margin + self.delta + other.delta;
                            if (separated(q.box, other.plan.sets[mc.variant].chunks[mc.chunk], m)) {
                                continue;
                            }
                            const SampleSet& reserved = other.plan.sets[mc.variant];
                            const size_t lo = std::max(mc.chunk * kChunk,
                                                       other.plan.cursor[mc.variant] > kBehind
                                                           ? other.plan.cursor[mc.variant] - kBehind
                                                           : size_t{0});
                            const size_t hi =
                                std::min({(mc.chunk + 1) * kChunk, reserved.pts.size(),
                                          other.plan.hi[mc.variant]});
                            for (size_t r = lo; r < hi; ++r) {
                                if (!sampleClear(self, q, other, reserved.pts[r], m)) {
                                    who = other.id;
                                    return true;
                                }
                            }
                        }
                        return false;
                    };
                    std::string who;
                    if (reservedConflict(true, who)) {
                        result.hard = true;
                        result.s = q.s;
                        result.blockers = {who};
                        return result;
                    }
                    if (soft_found) {
                        continue;
                    }
                    const bool bad = reservedConflict(false, who);
                    if (bad) {
                        soft_found = true;
                        if (q.s < result.s) {
                            result.s = q.s;
                            result.blockers = {who};
                        } else if (q.s == result.s) {
                            result.blockers.push_back(who);
                        }
                    }
                }
            }
        }
        return result;
    }

    // ----------------------------------------------------------------- planning one route

    std::vector<rc::Point> viaOf(const VisibilityPath& path) const {
        std::vector<rc::Point> via;
        for (size_t i = 1; i + 1 < path.points.size(); ++i) {
            via.push_back({path.points[i].x(), path.points[i].y()});
        }
        return via;
    }

    static ConvexObstacle obstacleOf(const Body& body, const Pose& p, const std::string& id) {
        Verts v;
        place(body.shape, p.x, p.y, p.yaw, &v);
        ConvexObstacle obstacle;
        obstacle.id = id;
        for (int k = 0; k < v.n; ++k) {
            obstacle.vertices.emplace_back(v.x[k], v.y[k]);
        }
        return obstacle;
    }

    // Routing-only outer rectangle for a frozen retiring sweep. It generates search vertices;
    // it does not replace or enlarge the Plan's reservation or its exact sampled verifier.
    static ConvexObstacle retiringRoutingObstacle(const Body& body) {
        std::vector<Eigen::Vector2d> points;
        auto append = [&](const Pose& pose) {
            Verts verts;
            place(body.shape, pose.x, pose.y, pose.yaw, &verts);
            for (int k = 0; k < verts.n; ++k) {
                points.emplace_back(verts.x[k], verts.y[k]);
            }
        };
        append(body.pose);  // the original verifier also checks the current reported footprint
        for (size_t variant = 0; variant < body.plan.sets.size(); ++variant) {
            const auto& set = body.plan.sets[variant];
            const size_t hi = std::min(set.pts.size(), body.plan.hi[variant]);
            const size_t lo =
                body.plan.cursor[variant] > kBehind ? body.plan.cursor[variant] - kBehind : 0;
            if (hi <= lo) {
                continue;  // the exact existing retained interval is empty
            }
            for (size_t i = lo; i < hi; ++i) {
                const auto& q = set.pts[i];
                append({q.x, q.y, q.yaw});
            }
        }
        // Four support halfspaces aligned with the frozen nominal route chord: a routing-only
        // outer rectangle with bounded visibility vertices. Every retained variant/corner is
        // projected; neither downsampling nor a last-pose/last-ack prefix substitutes for it.
        const double angle = std::atan2(body.plan.goal.y - body.plan.route.start().y,
                                        body.plan.goal.x - body.plan.route.start().x);
        const Eigen::Vector2d u(std::cos(angle), std::sin(angle)), v(-u.y(), u.x());
        double xmin = kInf, xmax = -kInf, ymin = kInf, ymax = -kInf;
        for (const auto& p : points) {
            const double x = u.dot(p), y = v.dot(p);
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
        }
        std::vector<Eigen::Vector2d> hull = {xmin * u + ymin * v, xmax * u + ymin * v,
                                             xmax * u + ymax * v, xmin * u + ymax * v};
        ConvexObstacle obstacle;
        obstacle.id = "frozen" + body.id;
        obstacle.vertices = std::move(hull);
        return obstacle;
    }

    const std::vector<ConvexObstacle>& staticObstacles() const {
        return static_list;
    }

    // Route for `body` from its measured pose to `goal`, verified exactly against everybody.
    // Two routing envelopes among the statics are tried (tight, then conservative), then two with
    // the peers' bodies as obstacles; the verifier is the only safety authority.
    Candidate planRoute(const Body& body, const Pose& goal, bool final_move) const {
        Candidate cand;
        const bool unicycle = body.spec.model == rc::Model::Unicycle;
        const double rho = body.shape.radius;
        struct Attempt {
            bool peers;
            double radius;
            double clearance;
        };
        // The last two are for a vehicle that starts close to a peer (inside the clearance the
        // earlier ones demand, but not in contact): a thin routing envelope finds the way out and
        // the exact verifier decides whether the body sweep really clears.
        const std::vector<Attempt> attempts = {
            {false, rho + 0.05, cfg.static_margin},
            {false, rho + (unicycle ? body.spec.ell_bound : 0.05), cfg.static_margin},
            {true, std::min(body.spec.footprint.half_width, body.spec.footprint.half_length) + 0.05,
             cfg.static_margin},
            {true, rho + 0.05, cfg.static_margin},
            {true, 0.5 * std::min(body.spec.footprint.half_width, body.spec.footprint.half_length),
             0.04},
            {true, 0.0, 0.02}};
        const std::vector<ConvexObstacle>& base = staticObstacles();
        // Escape certificates are attempt-independent geometry: which parked
        // peers the route's start pose overlaps inside the pair margin, the
        // inherited gap, and the separating halfspace each admission owns.
        // A start at contact/penetration (d0 <= 0) is a hard reject of every
        // candidate; a start at or beyond the margin needs no certificate.
        const double m_parked = cfg.pair_margin + body.delta;
        std::vector<EscapeCert> cert_base;
        std::string cert_contact;
        for (const auto& entry : bodies) {
            const Body& other = entry.second;
            if (&other == &body || other.plan_active) {
                continue;
            }
            Verts va0, vb;
            place(body.shape, body.pose.x, body.pose.y, body.pose.yaw, &va0);
            place(other.shape, other.pose.x, other.pose.y, other.pose.yaw, &vb);
            if (separation(va0.view(), vb.view(), m_parked) >= m_parked) {
                continue;
            }
            const ClosestPair cp = closestPair(va0.view(), vb.view());
            if (cp.d <= 0.0) {
                cert_contact = other.id;
                break;
            }
            if (cp.d >= m_parked) {
                continue;
            }
            EscapeCert cert;
            cert.peer_id = other.id;
            cert.peer_pose = other.pose;
            cert.peer_verts = vb;
            cert.d0 = cp.d;
            cert.body_fixed = cp.peer_v_to_self_e;
            if (!cert.body_fixed) {
                // The closest feature is a self vertex or a vertex-vertex
                // pair: the separating halfspace lives in the world frame.
                cert.nx = cp.nx;
                cert.ny = cp.ny;
                double mx = -kInf;
                for (int k = 0; k < vb.n; ++k) {
                    mx = std::max(mx, cp.nx * vb.x[k] + cp.ny * vb.y[k]);
                }
                cert.rhs = mx + cp.d;
            } else {
                // A peer vertex touches the interior of a self edge: the world
                // tangent locks every first rotation, so the halfspace is
                // attached to self's body frame instead - self's edge is then
                // stationary and only the peer's vertices swing.
                const double cy = std::cos(body.pose.yaw), sy = std::sin(body.pose.yaw);
                cert.nx = -cp.nx * cy - cp.ny * sy;
                cert.ny = cp.nx * sy - cp.ny * cy;
                const int nv = body.shape.rect ? 4 : 16;
                double mx = -kInf;
                for (int k = 0; k < nv; ++k) {
                    mx = std::max(mx, cert.nx * body.shape.bx[k] +
                                          cert.ny * body.shape.by[k]);
                }
                cert.rhs = mx + cp.d;
            }
            cert_base.push_back(std::move(cert));
        }
        bool static_route_found = false;
        for (size_t a = 0; a < attempts.size(); ++a) {
            const Attempt& at = attempts[a];
            std::vector<ConvexObstacle> obstacles = base;
            if (at.peers) {
                for (const auto& entry : bodies) {
                    const Body& other = entry.second;
                    if (&other == &body) {
                        continue;
                    }
                    if (other.plan_active && other.plan_retiring) {
                        obstacles.push_back(retiringRoutingObstacle(other));
                    } else if (other.plan_active) {
                        // Coarse stand-in for the remaining sweep (the exact verifier follows).
                        const SampleSet& set =
                            other.plan.sets.size() == 3 ? other.plan.sets[1] : other.plan.sets[0];
                        const size_t stride = static_cast<size_t>(std::ceil(1.0 / cfg.step));
                        const size_t lo = other.plan.cursor[0];
                        const size_t hi = std::min(
                            set.pts.size(), other.plan.hi[other.plan.sets.size() == 3 ? 1 : 0]);
                        for (size_t i = lo; i < hi; i += stride) {
                            obstacles.push_back(
                                obstacleOf(other, {set.pts[i].x, set.pts[i].y, set.pts[i].yaw},
                                           "r" + other.id));
                        }
                        if (hi > lo) {
                            obstacles.push_back(obstacleOf(
                                other, {set.pts[hi - 1].x, set.pts[hi - 1].y, set.pts[hi - 1].yaw},
                                "r" + other.id));
                        }
                    } else {
                        obstacles.push_back(obstacleOf(other, other.pose, "p" + other.id));
                    }
                }
            }
            const VisibilityPath path = planVisibilityPath(
                Eigen::Vector2d(body.pose.x, body.pose.y), Eigen::Vector2d(goal.x, goal.y),
                obstacles, fence, at.radius, at.clearance);
            if (!path.valid) {
                cand.why = "route: " + path.message;
                continue;
            }
            if (!at.peers) {
                static_route_found = true;
            }
            const std::vector<rc::Point> via = viaOf(path);
            if (via.size() > rc::kMaxVia) {
                cand.why = "route: more than " + std::to_string(rc::kMaxVia) + " via points";
                continue;
            }
            const double lc = body.ell_spread >= 0.0 ? body.ell_hat : 0.0;
            const double ls = body.ell_spread >= 0.0 ? body.ell_spread : body.spec.ell_bound;
            const std::vector<double> ells =
                unicycle ? std::vector<double>{lc - ls, lc, lc + ls} : std::vector<double>{0.0};
            Plan plan;
            plan.route = Route(body.pose, via, goal, unicycle ? lc : 0.0);
            plan.via = via;
            plan.goal = goal;
            plan.final_move = final_move;
            std::vector<const SampleSet*> view;
            for (double ell : ells) {
                const Route variant(body.pose, via, goal, ell);
                plan.sets.push_back(makeSet(variant, body.shape, unicycle, cfg));
                plan.total = std::max(
                    plan.total, plan.sets.back().pts.empty() ? 0.0 : plan.sets.back().pts.back().s);
            }
            for (const SampleSet& s : plan.sets) {
                view.push_back(&s);
            }
            if (!cert_contact.empty()) {
                cand.why = "sweep: parked body in contact at s=0";
                cand.blockers = {cert_contact};
                continue;
            }
            // Escape certificates become the admission's owned state. The
            // frozen escape index is derived once here - the first sample
            // whose swept pose clears the parked peer by the full margin - so
            // the verifier never replays the prefix.
            plan.escape_certs = cert_base;
            for (EscapeCert& cert : plan.escape_certs) {
                cert.escaped_at.assign(plan.sets.size(), kNoEscape);
                const auto peer = bodies.find(cert.peer_id);
                if (peer == bodies.end() || !samePose(cert.peer_pose, peer->second.pose)) {
                    continue;
                }
                const Sample at = poseSample(peer->second, cert.peer_pose);
                for (size_t v = 0; v < plan.sets.size(); ++v) {
                    const SampleSet& vs = plan.sets[v];
                    for (size_t i = 0; i < vs.pts.size(); ++i) {
                        if (sampleClear(body, vs.pts[i], peer->second, at, m_parked)) {
                            cert.escaped_at[v] = i;
                            break;
                        }
                    }
                }
            }
            const Violation violation =
                firstViolation(body, view, std::vector<size_t>(view.size(), 0),
                               plan.escape_certs);
            if (violation.hard) {
                std::ostringstream text;
                text << "sweep: hard at s=" << violation.s;
                cand.why = text.str();
                cand.blockers = violation.blockers;
                continue;
            }
            if (violation.s < kInf && violation.s - cfg.hold_margin < cfg.min_prefix) {
                std::ostringstream text;
                text << "sweep: soft too early at s=" << violation.s;
                cand.why = text.str();
                cand.blockers = violation.blockers;
                continue;
            }
            plan.s_limit = violation.s < kInf ? violation.s - cfg.hold_margin : 1e18;
            cand.ok = true;
            cand.plan = std::move(plan);
            cand.blockers.clear();
            return cand;
        }
        cand.route_failed = !static_route_found;
        return cand;
    }

    // Predicted duration of the own route at 70% of the declared speeds, plus settling.
    double predictedDuration(const Body& body, const Plan& plan) const {
        double turn = 0.0;
        double heading = body.pose.yaw;
        if (body.spec.model == rc::Model::Unicycle) {
            for (size_t j = 0; j < plan.route.segments(); ++j) {
                turn += std::abs(wrap(plan.route.heading(j) - heading));
                heading = plan.route.heading(j);
            }
        }
        turn += std::abs(wrap(plan.goal.yaw - heading));
        return plan.route.length() / (0.7 * body.spec.max_v) + turn / (0.7 * body.spec.max_w) + 3.0;
    }

    bool installPlan(Body& body, Plan plan) {
        if (body.grants >= 3) {
            emitReject(body, Reason::NoRoute, "Reset replan budget exhausted", {});
            return false;
        }
        plan.session = body.session;
        plan.request_generation = body.request_generation;
        body.plan = std::move(plan);
        body.plan_active = true;
        body.plan.grant_id = allocate();
        body.plan.cursor.assign(body.plan.sets.size(), 0);
        updateHi(body);
        body.hold_reason = Reason::None;
        body.blockers.clear();
        body.no_route_since = -1.0;
        body.progress_pose = body.pose;
        ++body.grants;
        body.plan_retiring = false;
        body.go_received = false;
        return true;
    }

    static void updateHi(Body& body) {
        Plan& plan = body.plan;
        plan.hi.resize(plan.sets.size());
        for (size_t w = 0; w < plan.sets.size(); ++w) {
            const auto& pts = plan.sets[w].pts;
            size_t lo = 0, hi = pts.size();
            while (lo < hi) {
                const size_t mid = (lo + hi) / 2;
                if (pts[mid].s <= plan.s_limit + 1e-9) {
                    lo = mid + 1;
                } else {
                    hi = mid;
                }
            }
            plan.hi[w] = lo;
        }
    }

    // ----------------------------------------------------------------- decisions

    PlannerDecision base(const Body& body, GrantState state) const {
        PlannerDecision d;
        d.id = body.id;
        d.session = body.session;
        d.request_generation = body.request_generation;
        d.state = state;
        return d;
    }

    void emitGo(Body& body) {
        PlannerDecision d = base(body, GrantState::Go);
        d.grant_id = body.plan.grant_id;
        d.start = body.plan.route.start();
        d.goal = body.plan.goal;
        d.final_move = body.plan.final_move;
        d.ell_plan = body.plan.route.ell();
        d.via = body.plan.via;
        d.s_limit = body.plan.s_limit;
        body.emitted = true;
        body.last_state = GrantState::Go;
        body.last_reason = Reason::None;
        body.last_grant = body.plan.grant_id;
        body.last_limit = body.plan.s_limit;
        body.last_blockers.clear();
        out.push_back(std::move(d));
    }

    void emitHold(Body& body, Reason reason, const std::string& detail,
                  const std::vector<std::string>& blockers) {
        body.hold_reason = reason;
        body.blockers = blockers;
        if (body.emitted && body.last_state == GrantState::Hold && body.last_reason == reason &&
            body.last_blockers == blockers) {
            return;
        }
        PlannerDecision d = base(body, GrantState::Hold);
        d.grant_id = allocate();
        d.reason = reason;
        d.detail = detail;
        d.blockers = blockers;
        body.emitted = true;
        body.last_state = GrantState::Hold;
        body.last_reason = reason;
        body.last_blockers = blockers;
        out.push_back(std::move(d));
    }

    void emitBrake(Body& body, Reason reason, const std::string& detail,
                   const std::vector<std::string>& blockers) {
        PlannerDecision d = base(body, GrantState::Brake);
        d.grant_id = allocate();
        d.reason = reason;
        d.detail = detail;
        d.blockers = blockers;
        body.awaiting_stop = true;
        body.certified = false;
        body.plan_retiring = body.plan_active;
        body.go_received = false;
        body.barrier_id = d.grant_id;
        body.emitted = true;
        body.last_state = GrantState::Brake;
        body.last_reason = reason;
        body.last_blockers = blockers;
        out.push_back(std::move(d));
    }

    void emitReject(Body& body, Reason reason, const std::string& detail,
                    const std::vector<std::string>& blockers) {
        PlannerDecision d = base(body, GrantState::Reject);
        d.grant_id = allocate();
        d.reason = reason;
        d.detail = detail;
        d.blockers = blockers;
        body.plan_retiring = body.plan_active;
        body.go_received = false;
        body.rejected = true;
        body.emitted = true;
        body.last_state = GrantState::Reject;
        body.last_reason = reason;
        body.last_blockers = blockers;
        ++stats.rejections;
        out.push_back(std::move(d));
    }

    // ----------------------------------------------------------------- synchronisation

    // Whether the body's current footprint reaches any granted, still-uncrossed
    // swept corridor, at that mover's own parked-peer margin - the same
    // broadphase the sweep applies. It is the relevance gate for changed-fact
    // invalidation: a quiescent body whose pose is stale against the last
    // verified anchor forces a recheck only where a certified route can be
    // affected.
    bool inGrantedCorridor(const Body& peer) const {
        Verts v;
        place(peer.shape, peer.pose.x, peer.pose.y, peer.pose.yaw, &v);
        const Aabb box = boxOf(v);
        for (const auto& entry : bodies) {
            const Body& other = entry.second;
            if (!other.plan_active || other.plan_retiring) {
                continue;
            }
            const Plan& plan = other.plan;
            const double m = cfg.pair_margin + other.delta;
            for (size_t w = 0; w < plan.sets.size(); ++w) {
                const SampleSet& s = plan.sets[w];
                const size_t hi = std::min(plan.hi[w], s.pts.size());
                for (size_t c = plan.cursor[w] / kChunk;
                     c < s.chunks.size() && c * kChunk < hi; ++c) {
                    if (!separated(s.chunks[c], box, m)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void sync(const std::vector<PlannerVehicle>& vehicles) {
        for (const PlannerVehicle& v : vehicles) {
            auto found = bodies.find(v.id);
            const bool fresh = found == bodies.end();
            Body& body = bodies[v.id];
            if (fresh) {
                body.id = v.id;
                body.since = now;
            }
            if (!fresh &&
                (v.request_generation < body.request_generation ||
                 (v.request_generation == body.request_generation && v.session != body.session))) {
                continue;
            }
            const bool new_request = v.request_generation > body.request_generation;
            const bool new_session = v.session != body.session;
            // A body that is not a granted mover is stale for proof purposes
            // when its reported pose differs from the anchor the last route
            // verification consumed (proof_pose, re-anchored only after a sweep
            // or admission ran) - or when no verification has covered it yet.
            // Both cases invalidate only inside a granted corridor: drift into
            // or appearance inside one raises the same flag setWorld does;
            // drift and new bodies elsewhere do not force a recheck.
            const bool stale_quiescent =
                (v.available || fresh) && !(body.plan_active && !body.plan_retiring) &&
                (!body.proof_pose_set || !samePose(v.pose, body.proof_pose));
            if (v.available || fresh) {
                body.pose = v.pose;
            }
            body.phase = v.phase;
            body.participant = v.phase == Phase::Requesting || v.phase == Phase::Waiting ||
                               v.phase == Phase::Moving || v.phase == Phase::Settling;
            if (!body.plan_active && (v.available || fresh)) {
                body.spec = v.spec;
                body.shape = makeShape(v.spec);
                setDelta(body);
            }
            // A quiescent body whose report drifted from its own anchor is a
            // pending change: inside a granted corridor it raises the flag
            // the extension sweep consumes; outside every corridor the
            // exclusion itself discharges it. The anchor moves only here -
            // no unrelated sweep or admission can erase what was captured.
            if (stale_quiescent) {
                if (inGrantedCorridor(body)) {
                    world_changed = true;
                    dirty = true;
                }
                body.proof_pose = v.pose;
                body.proof_pose_set = true;
            }
            body.ell_hat = v.ell_hat;
            body.ell_spread = v.ell_spread;
            body.uncontrolled = v.uncontrolled_moving ||
                                (body.participant && !body.plan_active && v.phase == Phase::Moving);
            if (new_request) {
                body.session = v.session;
                body.request_generation = v.request_generation;
                body.target = v.target;
                if (new_session || fresh) {
                    body.since = now;
                }
                body.rejected = false;
                body.emitted = false;
                body.occupied_since = -1.0;
                body.no_route_since = -1.0;
                emitBrake(body, Reason::Replan, "awaiting exact post-zero stopping certificate",
                          {});
                dirty = true;
            } else if (fresh) {
                body.session = v.session;
                body.target = v.target;
            }
            if (body.request_generation == 0) {
                continue;  // discovery only; no authority
            }
            if (body.plan_active && (!v.available || !body.participant) && !body.plan_retiring) {
                emitBrake(body, Reason::Replan,
                          "permission retained until stopping is acknowledged", {});
                dirty = true;
            }
            if (!new_request && v.available && body.awaiting_stop && v.stop_certified &&
                v.acknowledged_grant_id == body.barrier_id && body.barrier_id != 0) {
                // The only reservation retirement point. The latest declaration may now replace
                // the frozen old shape/spec; a repeated certificate cannot clear a later Plan.
                body.plan_active = false;
                body.plan_retiring = false;
                body.awaiting_stop = false;
                body.certified = true;
                body.go_received = false;
                body.spec = v.spec;
                body.shape = makeShape(v.spec);
                setDelta(body);
                body.target = v.target;
                if (body.budget_session != v.session) {
                    body.budget_session = v.session;
                    body.grants = 0;
                }
                body.emitted = false;
                if (!body.participant) {
                    emitHold(body, Reason::None, "Reset permission retired", {});
                }
                dirty = true;
            }
            if (body.plan_active && !body.plan_retiring && !v.stop_certified &&
                v.acknowledged_grant_id == body.plan.grant_id && body.plan.session == v.session &&
                body.plan.request_generation == v.request_generation) {
                body.go_received = true;
            }
        }
    }

    void setDelta(Body& body) const {
        const double reach = body.shape.radius +
                             (body.spec.model == rc::Model::Unicycle ? body.spec.ell_bound : 0.0);
        body.delta = ResetPlanner::samplingError(cfg, reach);
    }

    // ----------------------------------------------------------------- movers

    static void advanceCursor(Body& body) {
        Plan& plan = body.plan;
        for (size_t w = 0; w < plan.sets.size(); ++w) {
            const auto& pts = plan.sets[w].pts;
            size_t best = plan.cursor[w];
            double best_d = kInf;
            // Unicycle turns repeat positions; Omni's reserved circumscribed disc is heading
            // independent.
            for (size_t m = plan.cursor[w]; m < std::min(pts.size(), plan.cursor[w] + kWindow);
                 ++m) {
                const double d = std::hypot(pts[m].x - body.pose.x, pts[m].y - body.pose.y) +
                                 (body.spec.model == rc::Model::Unicycle
                                      ? 0.6 * std::abs(wrap(pts[m].yaw - body.pose.yaw))
                                      : 0.0);
                if (d < best_d) {
                    best_d = d;
                    best = m;
                }
            }
            plan.cursor[w] = best;
        }
    }

    double progressOf(const Body& body) const {
        const Plan& plan = body.plan;
        const size_t w = plan.sets.size() == 3 ? 1 : 0;
        return plan.sets[w].pts.empty()
                   ? 0.0
                   : plan.sets[w].pts[std::min(plan.cursor[w], plan.sets[w].pts.size() - 1)].s;
    }

    void updateMovers() {
        bool any_mover = false;
        bool advanced = false;
        const bool extend_due = now - last_extension >= cfg.extension_period;
        for (auto& entry : bodies) {
            Body& body = entry.second;
            if (!body.plan_active) {
                continue;
            }
            any_mover = true;
            if (body.plan_retiring || !body.go_received) {
                continue;
            }
            advanceCursor(body);
            if (std::hypot(body.pose.x - body.progress_pose.x, body.pose.y - body.progress_pose.y) >
                    0.02 ||
                std::abs(wrap(body.pose.yaw - body.progress_pose.yaw)) > 0.02) {
                body.progress_pose = body.pose;
                advanced = true;
            }
            // Whole-route parking completion is a vehicle-initiated new-G transaction.
            // Phase/proximity never retire the issued sweep here.
        }
        if (advanced || !any_mover) {
            fleet_progress = now;
        }
        if (!extend_due) {
            return;
        }
        last_extension = now;
        // Extension and revocation, in id order.
        for (auto& entry : bodies) {
            Body& body = entry.second;
            if (!body.plan_active) {
                continue;
            }
            if (body.plan_retiring) {
                continue;
            }
            const Plan& plan = body.plan;
            const bool prefix_open = plan.s_limit < plan.total - 1e-9;
            if (!prefix_open && !world_changed) {
                continue;
            }
            std::vector<const SampleSet*> rest;
            for (const SampleSet& s : plan.sets) {
                rest.push_back(&s);
            }
            const Violation violation =
                firstViolation(body, rest, plan.cursor, plan.escape_certs);
            const bool held = body.phase == Phase::Waiting;
            if (violation.hard && (held || world_changed)) {
                // The rest of the frozen route is blocked by a body that will not go away
                // (everyone else parked, or the scene changed): revoke at the hold point and
                // re-plan from where the vehicle stands, instead of waiting for it forever.
                emitBrake(body, violation.world ? Reason::SceneChanged : Reason::Replan,
                          violation.world ? "the scene changed under the route"
                                          : "the rest of the route is blocked",
                          violation.blockers);
                ++stats.revocations;
                dirty = true;
                continue;
            }
            if (violation.hard || !prefix_open) {
                continue;
            }
            if (!body.go_received) {
                continue;  // safety revocation above never waits for a Go receipt
            }
            const double next =
                violation.s == kInf ? 1e18 : std::max(plan.s_limit, violation.s - cfg.hold_margin);
            if (next > plan.s_limit + 1e-9) {
                body.plan.s_limit = next;
                updateHi(body);
                body.plan.grant_id = allocate();
                body.go_received = false;
                ++stats.extensions;
                emitGo(body);
            } else {
                body.blockers = violation.blockers;
            }
        }
        world_changed = false;
        // Fleet-wide stall: movers exist, none has advanced for stall_s.
        bool any = false;
        for (const auto& entry : bodies) {
            any = any || entry.second.plan_active;
        }
        if (any && now - fleet_progress > cfg.stall_s) {
            for (auto& entry : bodies) {
                Body& body = entry.second;
                if (!body.plan_active || body.plan_retiring || body.phase != Phase::Waiting) {
                    continue;
                }
                emitReject(body, Reason::Stalled, "no vehicle advanced for 10 s", body.blockers);
            }
            fleet_progress = now;
            dirty = true;
        }
    }

    // ----------------------------------------------------------------- admission

    bool targetBlockedByStatic(const Body& body) const {
        return !poseClearOfStatics(body, body.target, cfg.static_margin + 0.0);
    }

    // Admission checks of a waiting vehicle. Returns true if it was rejected (for itself only).
    bool checkSession(Body& body) {
        const bool finite = std::isfinite(body.pose.x) && std::isfinite(body.pose.y) &&
                            std::isfinite(body.pose.yaw) && std::isfinite(body.target.x) &&
                            std::isfinite(body.target.y) && std::isfinite(body.target.yaw);
        if (!finite) {
            emitReject(body, Reason::BadSession, "the pose or the target is not finite", {});
            return true;
        }
        Verts v;
        place(body.shape, body.target.x, body.target.y, body.target.yaw, &v);
        if (outsideFence(v, fence, cfg.fence_margin)) {
            emitReject(body, Reason::BadSession, "the target is outside the fence", {"fence"});
            return true;
        }
        std::string blocker;
        if (!poseClearOfStatics(body, body.target, cfg.static_margin, &blocker)) {
            emitReject(body, Reason::GoalOccupied, "the target is blocked by " + blocker,
                       {blocker});
            return true;
        }
        // A body outside any session that sits on the target blocks it, if it stays.
        std::vector<std::string> sitting;
        for (auto& entry : bodies) {
            Body& other = entry.second;
            if (&other == &body || other.participant) {
                continue;
            }
            if (!sampleClear(body, poseSample(body, body.target), other,
                             poseSample(other, other.pose), cfg.pair_margin)) {
                sitting.push_back(other.id);
            }
        }
        if (sitting.empty()) {
            body.occupied_since = -1.0;
        } else {
            if (body.occupied_since < 0.0) {
                body.occupied_since = now;
            }
            if (now - body.occupied_since >= cfg.occupancy_settle_s) {
                emitReject(body, Reason::GoalOccupied, "the target is occupied by " + sitting[0],
                           sitting);
                return true;
            }
            emitHold(body, Reason::WaitingForPeer, "the target is occupied by " + sitting[0],
                     sitting);
        }
        return false;
    }

    std::vector<Eigen::Vector2d> staticRoute(const Body& body, const Pose& goal) const {
        const VisibilityPath path = planVisibilityPath(
            Eigen::Vector2d(body.pose.x, body.pose.y), Eigen::Vector2d(goal.x, goal.y),
            staticObstacles(), fence, body.shape.radius + 0.05, cfg.static_margin);
        if (path.valid) {
            return path.points;
        }
        return {Eigen::Vector2d(body.pose.x, body.pose.y), Eigen::Vector2d(goal.x, goal.y)};
    }

    static double polylineDistance(const std::vector<Eigen::Vector2d>& line, double px, double py) {
        double best = kInf;
        for (size_t i = 0; i + 1 < line.size(); ++i) {
            best = std::min(best, segmentDistance(px, py, line[i].x(), line[i].y(), line[i + 1].x(),
                                                  line[i + 1].y()));
        }
        return best;
    }

    // Ordering rule: defer k while another waiting vehicle's static route passes through k's goal
    // ("that one passes first") or through k's route runs through a start whose owner does not run
    // back through it ("it leaves first"). Returns the vehicles it waits for.
    std::vector<std::string> deferredBy(
        const Body& k, const std::vector<Body*>& waiting,
        const std::map<std::string, std::vector<Eigen::Vector2d>>& routes) const {
        std::vector<std::string> by;
        for (const Body* j : waiting) {
            if (j == &k || j->rejected) {
                continue;
            }
            const double clearance = j->shape.radius + k.shape.radius + cfg.pair_margin;
            if (polylineDistance(routes.at(j->id), k.target.x, k.target.y) < clearance) {
                by.push_back(j->id);
                continue;
            }
            if (polylineDistance(routes.at(k.id), j->pose.x, j->pose.y) < clearance &&
                !(polylineDistance(routes.at(j->id), k.pose.x, k.pose.y) < clearance)) {
                by.push_back(j->id);
            }
        }
        return by;
    }

    bool anyMover() const {
        for (const auto& entry : bodies) {
            if (entry.second.plan_active && !entry.second.plan_retiring) {
                return true;
            }
        }
        return false;
    }

    int admit() {
        std::vector<Body*> waiting;
        for (auto& entry : bodies) {
            Body& b = entry.second;
            if (b.participant && b.certified && !b.awaiting_stop && !b.plan_active && !b.rejected &&
                (b.phase == Phase::Requesting || b.phase == Phase::Waiting)) {
                waiting.push_back(&b);
            }
        }
        if (waiting.empty()) {
            return 0;
        }
        for (Body* b : waiting) {
            checkSession(*b);
        }
        // The goals of two vehicles of one Reset must be separated by the exact clearance; an
        // overlap is an authoring error and neither of the two can be right, so both are refused.
        {
            std::vector<Body*> all;
            for (auto& entry : bodies) {
                if (entry.second.participant && !entry.second.rejected) {
                    all.push_back(&entry.second);
                }
            }
            std::map<std::string, std::vector<std::string>> overlaps;
            for (size_t a = 0; a < all.size(); ++a) {
                for (size_t c = a + 1; c < all.size(); ++c) {
                    if (!sampleClear(*all[a], poseSample(*all[a], all[a]->target), *all[c],
                                     poseSample(*all[c], all[c]->target), cfg.pair_margin)) {
                        overlaps[all[a]->id].push_back(all[c]->id);
                        overlaps[all[c]->id].push_back(all[a]->id);
                    }
                }
            }
            for (auto& entry : overlaps) {
                Body& b = bodies[entry.first];
                if (!b.rejected && !b.plan_active) {
                    emitReject(b, Reason::GoalOccupied,
                               "the target overlaps the target of " + entry.second[0],
                               entry.second);
                }
            }
        }
        waiting.erase(std::remove_if(waiting.begin(), waiting.end(),
                                     [](const Body* b) { return b->rejected; }),
                      waiting.end());
        if (waiting.empty()) {
            return 0;
        }
        for (const auto& entry : bodies) {
            if (entry.second.uncontrolled) {
                std::vector<std::string> who;
                for (const auto& e : bodies) {
                    if (e.second.uncontrolled) {
                        who.push_back(e.first);
                    }
                }
                for (Body* b : waiting) {
                    emitHold(*b, Reason::NonParticipantMoving, "a body outside Reset is moving",
                             who);
                }
                return 0;
            }
        }
        std::map<std::string, std::vector<Eigen::Vector2d>> routes;
        for (Body* b : waiting) {
            routes[b->id] = staticRoute(*b, b->target);
        }
        int granted = 0;
        std::map<std::string, Candidate> failed;
        // Who each waiting vehicle defers to. A vehicle whose deferrers all wait themselves is in a
        // cycle of "that one passes first" (or behind one): the ordering rule can not break it, and
        // it would otherwise wait for every mover to finish. Such vehicles are planned without the
        // rule, in id order, as are all of them when nobody moves.
        std::map<std::string, std::vector<std::string>> deferrers;
        for (Body* b : waiting) {
            deferrers[b->id] = deferredBy(*b, waiting, routes);
        }
        std::set<std::string> in_cycle;
        for (Body* b : waiting) {
            const auto& by = deferrers[b->id];
            bool all_wait = !by.empty();
            for (const std::string& id : by) {
                all_wait = all_wait && deferrers.count(id) != 0 && !deferrers[id].empty();
            }
            if (all_wait) {
                in_cycle.insert(b->id);
            }
        }
        for (int pass = 0; pass < 2; ++pass) {
            if (pass == 1 && (granted > 0 || anyMover()) && in_cycle.empty()) {
                break;
            }
            for (Body* b : waiting) {
                if (b->plan_active || b->rejected) {
                    continue;
                }
                if (pass == 0) {
                    const std::vector<std::string>& by = deferrers[b->id];
                    if (!by.empty()) {
                        emitHold(*b, Reason::Precedence, "waiting for " + by[0] + " to pass", by);
                        continue;
                    }
                } else if (granted > 0 || anyMover()) {
                    // The fallback while others move: only the members of a cycle.
                    if (in_cycle.count(b->id) == 0) {
                        continue;
                    }
                }
                Candidate cand = planRoute(*b, b->target, true);
                if (!cand.ok) {
                    failed[b->id] = std::move(cand);
                    continue;
                }
                if (pass == 1 && (granted > 0 || anyMover()) &&
                    cand.plan.s_limit < std::min(cand.plan.total, kCycleReach)) {
                    // Breaking a cycle while others move is only worth a route that gets somewhere:
                    // a short prefix would only park the vehicle in the way of the others.
                    Candidate held;
                    held.why = "sweep: only a short prefix is free";
                    held.blockers = cand.blockers;
                    failed[b->id] = std::move(held);
                    continue;
                }
                const double predicted = predictedDuration(*b, cand.plan);
                if (predicted > cfg.refuse_factor * cfg.timeout) {
                    std::ostringstream text;
                    text << "the route needs about " << static_cast<int>(predicted)
                         << " s, more than " << cfg.refuse_factor << " x the " << cfg.timeout
                         << " s limit";
                    emitReject(*b, Reason::TooSlow, text.str(), {});
                    continue;
                }
                if (!installPlan(*b, std::move(cand.plan))) {
                    continue;
                }
                emitGo(*b);
                ++stats.admissions;
                ++granted;
            }
        }
        // Report why the others wait.
        for (Body* b : waiting) {
            if (b->plan_active || b->rejected) {
                continue;
            }
            auto f = failed.find(b->id);
            if (f == failed.end()) {
                continue;  // deferred: already told
            }
            const Candidate& c = f->second;
            if (c.route_failed) {
                if (b->no_route_since < 0.0) {
                    b->no_route_since = now;
                }
                if (now - b->no_route_since >= 5.0) {
                    emitReject(*b, Reason::NoRoute, c.why, {});
                    continue;
                }
                emitHold(*b, Reason::NoRoute, c.why, {});
            } else {
                b->no_route_since = -1.0;
                emitHold(*b, Reason::WaitingForPeer, c.why, c.blockers);
            }
        }
        return granted;
    }

    // Nobody moves and nobody can be granted: one parking move that makes progress, else report.
    bool park() {
        struct Choice {
            Body* body{nullptr};
            Pose pose;
            Candidate cand;
            int score{-1};
            double length{1e9};
        };
        Choice best;
        for (auto& entry : bodies) {
            Body& k = entry.second;
            if (!k.participant || !k.certified || k.awaiting_stop || k.plan_active || k.rejected) {
                continue;
            }
            for (double r : {1.6, 2.4, 3.2}) {
                for (int a = 0; a < 12; ++a) {
                    const double angle = 2.0 * rc::kPi * a / 12;
                    const Pose p{k.pose.x + r * std::cos(angle), k.pose.y + r * std::sin(angle),
                                 k.pose.yaw};
                    if (!poseClearOfStatics(k, p, cfg.static_margin)) {
                        continue;
                    }
                    bool ok = true;
                    for (const auto& e : bodies) {
                        const Body& j = e.second;
                        if (&j == &k || !ok) {
                            continue;
                        }
                        ok = sampleClear(k, poseSample(k, p), j, poseSample(j, j.pose),
                                         cfg.pair_margin);
                        // Do not park on a goal.
                        if (ok && j.participant) {
                            ok = sampleClear(k, poseSample(k, p), j, poseSample(j, j.target),
                                             cfg.pair_margin);
                        }
                    }
                    if (!ok) {
                        continue;
                    }
                    Candidate cand = planRoute(k, p, false);
                    if (!cand.ok) {
                        continue;
                    }
                    // Potential: the number of vehicles that can be granted once k sits at p.
                    const Pose saved = k.pose;
                    k.pose = p;
                    int score = 0;
                    for (auto& e : bodies) {
                        Body& j = e.second;
                        if (!j.participant || j.rejected || &j == &k) {
                            continue;
                        }
                        if (planRoute(j, j.target, true).ok) {
                            ++score;
                        }
                    }
                    k.pose = saved;
                    const double length = cand.plan.route.length();
                    if (score > best.score || (score == best.score && length < best.length)) {
                        best.body = &k;
                        best.pose = p;
                        best.cand = std::move(cand);
                        best.score = score;
                        best.length = length;
                    }
                }
            }
        }
        if (best.body == nullptr || best.score <= 0) {
            return false;
        }
        if (!installPlan(*best.body, std::move(best.cand.plan))) {
            return false;
        }
        emitGo(*best.body);
        ++stats.parks;
        return true;
    }

    void deadlock() {
        std::vector<std::string> stuck;
        for (const auto& entry : bodies) {
            if (entry.second.participant && entry.second.certified && !entry.second.awaiting_stop &&
                !entry.second.plan_active && !entry.second.rejected) {
                stuck.push_back(entry.first);
            }
        }
        for (const std::string& id : stuck) {
            Body& b = bodies[id];
            std::vector<std::string> who = b.blockers;
            who.insert(who.end(), stuck.begin(), stuck.end());
            who.erase(std::remove(who.begin(), who.end(), id), who.end());
            std::sort(who.begin(), who.end());
            who.erase(std::unique(who.begin(), who.end()), who.end());
            emitReject(b, Reason::Deadlock, "nobody can move and no parking move helps", who);
        }
    }

    // ----------------------------------------------------------------- the step

    void step(bool admit_allowed) {
        if (!admit_allowed) {
            for (auto& entry : bodies) {
                Body& b = entry.second;
                if (b.plan_active && !b.plan_retiring) {
                    emitBrake(b, Reason::SceneUnavailable, "new admissions unavailable", {});
                }
            }
        }
        updateMovers();
        if (!admit_allowed) {
            return;
        }
        const bool periodic = now - last_replan >= cfg.replan_period;
        if (!dirty && !periodic) {
            return;
        }
        bool waiting = false;
        for (const auto& entry : bodies) {
            const Body& b = entry.second;
            waiting = waiting ||
                      (b.participant && b.certified && !b.awaiting_stop && !b.plan_active &&
                       !b.rejected && (b.phase == Phase::Requesting || b.phase == Phase::Waiting));
        }
        dirty = false;
        if (!waiting) {
            return;
        }
        last_replan = now;
        ++stats.events;
        const int granted = admit();
        if (granted == 0 && !anyMover()) {
            // Everyone waiting could not be admitted and nobody moves. A refusal for the
            // vehicle itself (rejected, held for a precedence or a moving outsider) is not a
            // deadlock: only vehicles that tried and failed are.
            bool blocked = false;
            for (const auto& entry : bodies) {
                const Body& b = entry.second;
                if (b.participant && b.certified && !b.awaiting_stop && !b.plan_active &&
                    !b.rejected && !b.uncontrolled &&
                    (b.phase == Phase::Requesting || b.phase == Phase::Waiting)) {
                    blocked = true;
                }
            }
            bool outsider = false;
            for (const auto& entry : bodies) {
                outsider = outsider || entry.second.uncontrolled;
            }
            if (blocked && !outsider && !park()) {
                // Hold reasons from this round decide: a vehicle held only by the target occupancy
                // settle time is not stuck yet.
                bool settling = false;
                for (const auto& entry : bodies) {
                    const Body& b = entry.second;
                    settling = settling || (b.participant && !b.plan_active && !b.rejected &&
                                            b.occupied_since >= 0.0);
                }
                bool no_route_pending = false;
                for (const auto& entry : bodies) {
                    const Body& b = entry.second;
                    no_route_pending = no_route_pending || (b.participant && !b.plan_active &&
                                                            !b.rejected && b.no_route_since >= 0.0);
                }
                if (!settling && !no_route_pending) {
                    deadlock();
                }
            }
        }
    }
};

// ---------------------------------------------------------------------------------------------

ResetPlanner::ResetPlanner(const PlannerConfig& config) : impl_(new Impl(config)) {}
ResetPlanner::~ResetPlanner() = default;
uint32_t ResetPlanner::allocateGrantId() {
    return impl_->allocate();
}

double ResetPlanner::samplingError(const PlannerConfig& config, double radius) {
    return std::max(0.5 * config.step, 0.5 * config.step_yaw * radius);
}

void ResetPlanner::setWorld(const std::vector<ConvexObstacle>& statics, const Fence& fence) {
    impl_->statics.clear();
    impl_->static_list.clear();
    for (const ConvexObstacle& o : statics) {
        StaticPoly p;
        p.id = o.id;
        for (const auto& v : o.vertices) {
            p.x.push_back(v.x());
            p.y.push_back(v.y());
            p.box.add(v.x(), v.y());
        }
        if (p.x.size() >= 3) {
            impl_->static_list.push_back(o);
            impl_->statics.push_back(std::move(p));
        }
    }
    impl_->fence = fence;
    impl_->world_changed = true;
    impl_->dirty = true;
}

std::vector<PlannerDecision> ResetPlanner::update(double now,
                                                  const std::vector<PlannerVehicle>& vehicles,
                                                  bool admit) {
    const auto t0 = std::chrono::steady_clock::now();
    Impl& m = *impl_;
    m.now = now;
    m.out.clear();
    m.sync(vehicles);
    m.step(admit);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    m.stats.plan_ms_max = std::max(m.stats.plan_ms_max, ms);
    m.stats.plan_ms_sum += ms;
    ++m.stats.plan_calls;
    return std::move(m.out);
}

std::vector<PlannerBodyReport> ResetPlanner::report() const {
    std::vector<PlannerBodyReport> list;
    for (const auto& entry : impl_->bodies) {
        const Body& b = entry.second;
        PlannerBodyReport r;
        r.id = b.id;
        r.participant = b.participant;
        r.has_route = b.plan_active;
        if (b.plan_active) {
            r.progress = impl_->progressOf(b);
            r.remaining = std::max(0.0, b.plan.route.length() - r.progress);
            r.s_limit = b.plan.s_limit;
        }
        r.state = b.emitted ? b.last_state : GrantState::Hold;
        r.reason = b.emitted ? b.last_reason : Reason::None;
        r.blockers = b.emitted ? b.last_blockers : std::vector<std::string>();
        list.push_back(std::move(r));
    }
    return list;
}

const PlannerStats& ResetPlanner::stats() const {
    return impl_->stats;
}

const PlannerConfig& ResetPlanner::config() const {
    return impl_->cfg;
}

}  // namespace ugv_reset_safety
