#pragma once
#include <array>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
namespace ugv_reset_safety {
// Fixed worker count and one coalescing pending bit per registered native
// solver. Work is registered at startup; submit creates no thread or queue node.
class FixedExecutor {
   public:
    static constexpr std::size_t capacity = 128;
    explicit FixedExecutor(std::size_t workers = 2) {
        if (!workers || workers > 4)
            throw std::invalid_argument("executor workers must be 1..4");
        try {
            for (std::size_t i = 0; i < workers; ++i)
                workers_.emplace_back([this] { run(); });
        } catch (...) {
            {
                std::lock_guard lock(mutex_);
                stopping_ = true;
            }
            changed_.notify_all();
            for (auto& w : workers_)
                w.join();
            throw;
        }
    }
    ~FixedExecutor() {
        {
            std::lock_guard lock(mutex_);
            stopping_ = true;
            for (auto& s : slots_)
                s.pending = false;
        }
        changed_.notify_all();
        for (auto& w : workers_)
            w.join();
    }
    FixedExecutor(const FixedExecutor&) = delete;
    FixedExecutor& operator=(const FixedExecutor&) = delete;
    void attach(std::size_t index, std::function<void()> work) {
        std::lock_guard lock(mutex_);
        auto& slot = slots_.at(index);
        if (stopping_ || slot.work || !work)
            throw std::invalid_argument("executor slot already owned");
        slot.work = std::move(work);
    }
    bool submit(std::size_t index) {
        std::lock_guard lock(mutex_);
        auto& slot = slots_.at(index);
        if (stopping_ || !slot.work || slot.retiring)
            return false;
        if (!slot.pending) {
            slot.pending = true;
            changed_.notify_one();
        }
        return true;
    }
    // A consumer retains its native solver until its fixed slot is quiescent.
    void detach(std::size_t index) {
        std::unique_lock lock(mutex_);
        auto& slot = slots_.at(index);
        slot.retiring = true;
        slot.pending = false;
        changed_.wait(lock, [&] { return !slot.running; });
        slot.work = {};
        slot.retiring = false;
    }

   private:
    struct Slot {
        std::function<void()> work;
        bool pending = false, running = false, retiring = false;
    };
    void run() {
        std::unique_lock lock(mutex_);
        for (;;) {
            changed_.wait(lock, [&] {
                if (stopping_)
                    return true;
                for (const auto& s : slots_)
                    if (s.pending && !s.running && !s.retiring)
                        return true;
                return false;
            });
            if (stopping_)
                return;
            std::size_t selected = capacity;
            for (std::size_t n = 0; n < capacity; ++n) {
                auto i = (cursor_ + n) % capacity;
                auto& s = slots_[i];
                if (s.pending && !s.running && !s.retiring) {
                    selected = i;
                    break;
                }
            }
            if (selected == capacity)
                continue;
            auto& slot = slots_[selected];
            slot.pending = false;
            slot.running = true;
            cursor_ = (selected + 1) % capacity;
            // Only detach can clear this startup-owned callable, after running=false.
            lock.unlock();
            try {
                slot.work();
            } catch (...) {
            }
            lock.lock();
            slot.running = false;
            changed_.notify_all();
        }
    }
    std::mutex mutex_;
    std::condition_variable changed_;
    std::array<Slot, capacity> slots_{};
    std::vector<std::thread> workers_;
    std::size_t cursor_ = 0;
    bool stopping_ = false;
};
}  // namespace ugv_reset_safety
