# UGV reset safety core

The ROS-free library contains the original geometry, path, DWA, fleet schedule,
ResetSession and native FleetCoordinator. It accepts explicit world and monotonic
clocks and plain scene/pose data. The single `ugv_fleet_host` embeds it with the
original native controller cores and sole command publishers; management uses
its bounded XRPC reset contract. See `../ugv_fleet_host/README.md`.

The math preserves original footprint, fence, clearance, uncertainty, velocity
and acceleration limits, 5 cm / 5 degree arrival checks, measured stop and actual
published-zero criteria. Requests and feedback retain exact generation/stamp
identity and 150 ms dual-clock leases. Scene revision and scientific clock epoch
changes stop an active batch. Original controller ResetState terminal events
remain distinct from Ready and require actual zero publication completion.

Configurations for four Scout, two Mecanum and mixed chassis fleets remain in
`config/`. Experiment slot-pose and world-boundary preparation preserve existing
controller tuning and exact private YAML namespaces. `ResetClient`, reset ROS
request/response messages and the separate ROS Coordinator are retired.
