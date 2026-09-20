#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

/**
 * A fixed-capacity single-producer/single-consumer queue.
 *
 * The producer and consumer only publish indices to each other. Queue
 * operations therefore never allocate, wait, disable interrupts, or take a
 * cross-core lock. Monotonic producer/consumer counters distinguish full from
 * empty while masked array indices avoid division and wrap branches.
 */
template <typename T, std::size_t Capacity>
class FixedSpscQueue {
    static_assert(Capacity > 0, "FixedSpscQueue capacity must not be zero");
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "FixedSpscQueue capacity must be a power of two");
    static_assert(Capacity <= UINT32_MAX / 2,
                  "FixedSpscQueue capacity is too large for its counters");

public:
    [[nodiscard]] bool try_push(const T& value)
    {
        return try_push_impl(value);
    }

    [[nodiscard]] bool try_push(T&& value)
    {
        return try_push_impl(std::move(value));
    }

    [[nodiscard]] bool try_pop(T& value)
    {
        const uint32_t read = read_count_.load(std::memory_order_relaxed);
        if (read == write_count_.load(std::memory_order_acquire)) {
            return false;
        }

        value = std::move(buffer_[read & index_mask]);
        read_count_.store(read + 1, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool empty() const
    {
        return read_count_.load(std::memory_order_acquire)
            == write_count_.load(std::memory_order_acquire);
    }

    [[nodiscard]] std::size_t size() const
    {
        const uint32_t write = write_count_.load(std::memory_order_acquire);
        const uint32_t read = read_count_.load(std::memory_order_acquire);
        return static_cast<std::size_t>(write - read);
    }

    [[nodiscard]] std::size_t available() const
    {
        return Capacity - size();
    }

    static constexpr std::size_t capacity()
    {
        return Capacity;
    }

    // Only call before the producer and consumer begin running.
    void reset()
    {
        read_count_.store(0, std::memory_order_relaxed);
        write_count_.store(0, std::memory_order_relaxed);
    }

private:
    template <typename U>
    bool try_push_impl(U&& value)
    {
        const uint32_t write = write_count_.load(std::memory_order_relaxed);
        const uint32_t read = read_count_.load(std::memory_order_acquire);
        if (write - read == Capacity) {
            return false;
        }

        buffer_[write & index_mask] = std::forward<U>(value);
        write_count_.store(write + 1, std::memory_order_release);
        return true;
    }

    static constexpr uint32_t index_mask = Capacity - 1;

    std::array<T, Capacity> buffer_ = {};
    alignas(4) std::atomic_uint32_t read_count_ = 0;
    alignas(4) std::atomic_uint32_t write_count_ = 0;
};

/**
 * A fixed-capacity queue for use by a single execution context.
 *
 * Pushing into a full queue overwrites the oldest entry, so the queue always
 * retains the most recent Capacity values. This queue is deliberately not
 * cross-core safe.
 */
template <typename T, std::size_t Capacity>
class FixedOverwriteQueue {
    static_assert(Capacity > 0, "FixedOverwriteQueue capacity must not be zero");
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "FixedOverwriteQueue capacity must be a power of two");

public:
    // Returns true when the oldest entry was overwritten.
    bool push(const T& value)
    {
        return push_impl(value);
    }

    // Returns true when the oldest entry was overwritten.
    bool push(T&& value)
    {
        return push_impl(std::move(value));
    }

    [[nodiscard]] const T& front() const
    {
        return buffer_[(write_count_ - count_) & index_mask];
    }

    [[nodiscard]] bool pop()
    {
        if (empty()) {
            return false;
        }

        --count_;
        return true;
    }

    [[nodiscard]] bool empty() const
    {
        return count_ == 0;
    }

    [[nodiscard]] bool full() const
    {
        return count_ == Capacity;
    }

    [[nodiscard]] std::size_t size() const
    {
        return count_;
    }

    static constexpr std::size_t capacity()
    {
        return Capacity;
    }

    void clear()
    {
        write_count_ = 0;
        count_ = 0;
    }

private:
    template <typename U>
    bool push_impl(U&& value)
    {
        const bool overwritten = full();
        buffer_[write_count_ & index_mask] = std::forward<U>(value);
        ++write_count_;
        count_ += static_cast<std::size_t>(!overwritten);

        return overwritten;
    }

    static constexpr std::size_t index_mask = Capacity - 1;

    std::array<T, Capacity> buffer_ = {};
    std::size_t write_count_ = 0;
    std::size_t count_ = 0;
};
