/**
 * @file spsc_ring_buffer.hpp
 * @brief Lock-free, wait-free ring buffer for Single-Producer / Single-Consumer (SPSC) use.
 *
 * Designed for real-time sensor pipelines where one thread publishes telemetry and a
 * second thread consumes it. The implementation avoids mutexes entirely; correctness
 * is guaranteed by careful use of C++11 @c std::atomic load/store with acquire-release
 * ordering and by keeping producer and consumer indices on separate cache lines.
 *
 * ### Concurrency contract
 * - Exactly **one** thread may call push() at any time (the producer).
 * - Exactly **one** thread may call pop()  at any time (the consumer).
 * - These may be two different threads running concurrently.
 * - Any other usage pattern is undefined behaviour.
 *
 * ### Capacity vs. allocation
 * Internally the buffer allocates @c capacity+1 slots. The extra slot implements
 * the classic "leave one slot empty" trick that distinguishes a full ring from an
 * empty one without a separate counter.
 */

#ifndef EDGE_SYNC_CONCURRENCY_SPSC_RING_BUFFER_HPP
#define EDGE_SYNC_CONCURRENCY_SPSC_RING_BUFFER_HPP

#include <atomic>
#include <cstddef>
#include <vector>

namespace edge_sync::concurrency
{

/// Cache-line size used for padding atomic indices. Falls back to 64 bytes when
/// the C++17 hardware interference constant is not available.
#ifdef __cpp_lib_hardware_interference_size
    using std::hardware_constructive_interference_size;
#else
    constexpr std::size_t hardware_constructive_interference_size = 64;
#endif

/**
 * @brief Lock-free SPSC ring buffer.
 *
 * @tparam T Element type. When storing sensor data, prefer a raw pointer
 *           (e.g., @c ImuMessage*) so that push/pop copy only one word.
 */
template<typename T>
class SpscRingBuffer
{
public:

    /**
     * @brief Constructs the ring buffer and pre-allocates all internal storage.
     *
     * The underlying vector is resized to @p capacity+1 immediately so that no
     * heap allocation occurs on the hot path.
     *
     * @param capacity Maximum number of elements the buffer can hold simultaneously.
     *                 Must be greater than zero.
     */
    explicit SpscRingBuffer(size_t capacity);

    SpscRingBuffer(const SpscRingBuffer&)            = delete; ///< Non-copyable.
    SpscRingBuffer(SpscRingBuffer&&)                 = delete; ///< Non-movable.
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete; ///< Non-copy-assignable.
    SpscRingBuffer& operator=(SpscRingBuffer&&)      = delete; ///< Non-move-assignable.

    /**
     * @brief Attempts to enqueue an element (producer side).
     *
     * Loads the current tail with `relaxed` ordering (safe — only the producer
     * ever writes the tail), then reads the head with `acquire` ordering to
     * synchronise with the consumer's last pop().
     *
     * Complexity: O(1), 0 heap allocations, wait-free.
     *
     * @param item Element to copy into the buffer.
     * @return @c true on success; @c false if the buffer is full.
     */
    bool push(const T& item);

    /**
     * @brief Attempts to dequeue an element (consumer side).
     *
     * Loads the current head with `relaxed` ordering (safe — only the consumer
     * ever writes the head), then reads the tail with `acquire` ordering to
     * synchronise with the producer's last push().
     *
     * Complexity: O(1), 0 heap allocations, wait-free.
     *
     * @param[out] outItem Receives the dequeued element on success. Left
     *                     unmodified when the buffer is empty.
     * @return @c true on success; @c false if the buffer is empty.
     */
    bool pop(T& outItem);

private:

    size_t m_capacity; ///< Internal capacity (user-requested capacity + 1).

    /// Pre-allocated ring storage. Sized once at construction; never reallocated.
    std::vector<T> m_buffer;

    // -----------------------------------------------------------------------
    // CACHE-LINE ISOLATION
    // Each index lives on its own cache line. Without this padding, both
    // atomics could share a line, causing false sharing: a producer write to
    // m_tail would invalidate the consumer's cached m_head, and vice-versa,
    // serialising what should be independent operations.
    // -----------------------------------------------------------------------

    /// Read by both threads; written exclusively by the consumer (pop).
    alignas(hardware_constructive_interference_size) std::atomic<size_t> m_head{0};

    /// Read by both threads; written exclusively by the producer (push).
    alignas(hardware_constructive_interference_size) std::atomic<size_t> m_tail{0};
};

template<typename T>
SpscRingBuffer<T>::SpscRingBuffer(size_t capacity) :
        m_capacity(capacity + 1) // +1 distinguishes full from empty without an extra counter
{
    m_buffer.resize(m_capacity);
}

template<typename T>
bool SpscRingBuffer<T>::push(const T& item)
{
    const size_t currentTail = m_tail.load(std::memory_order_relaxed);
    const size_t nextTail    = (currentTail + 1) % m_capacity;

    // Acquire-load of head synchronises with the release-store in pop(), ensuring
    // we observe the consumer's most recent read index before checking for space.
    if (nextTail == m_head.load(std::memory_order_acquire))
    {
        return false; // buffer full
    }

    m_buffer[currentTail] = item;

    // Release-store makes the written element visible to the consumer before the
    // updated tail is observed.
    m_tail.store(nextTail, std::memory_order_release);

    return true;
}

template<typename T>
bool SpscRingBuffer<T>::pop(T& outItem)
{
    const size_t currentHead = m_head.load(std::memory_order_relaxed);

    // Acquire-load of tail synchronises with the release-store in push(), ensuring
    // we observe the fully written element before reading it.
    if (currentHead == m_tail.load(std::memory_order_acquire))
    {
        return false; // buffer empty
    }

    outItem = m_buffer[currentHead];

    // Release-store makes the updated head index visible to the producer before it
    // checks for available space on the next push().
    m_head.store((currentHead + 1) % m_capacity, std::memory_order_release);

    return true;
}

} // namespace edge_sync::concurrency

#endif // EDGE_SYNC_CONCURRENCY_SPSC_RING_BUFFER_HPP
