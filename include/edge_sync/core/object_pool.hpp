/**
 * @file object_pool.hpp
 * @brief Zero-heap-allocation object pool backed by a single contiguous memory block.
 *
 * Provides O(1) acquire/release operations suitable for real-time hot paths where
 * dynamic allocation is prohibited. The pool is non-thread-safe by design; callers
 * must ensure exclusive access or wrap with an appropriate synchronisation primitive.
 */

#ifndef OBJECT_POOL_HPP
#define OBJECT_POOL_HPP

#include <cstddef>
#include <memory>
#include <new>

namespace edge_sync::core
{

/// Cache-line size used for aligning PoolNode. Falls back to 64 bytes when the
/// C++17 hardware interference constant is not available.
#ifdef __cpp_lib_hardware_interference_size
    using std::hardware_constructive_interference_size;
#else
    constexpr std::size_t hardware_constructive_interference_size = 64;
#endif

/**
 * @brief Intrusive linked-list node that wraps a single pooled object.
 *
 * Aligned to the hardware cache-line boundary so that adjacent nodes in the
 * contiguous memory block do not share a cache line, preventing false sharing
 * when different threads touch different nodes.
 *
 * @tparam T The type of the pooled object stored inside this node.
 */
template<typename T>
struct alignas(hardware_constructive_interference_size) PoolNode
{
    T data;                  ///< The actual object vended to callers.
    PoolNode<T>* next{nullptr}; ///< Intrusive pointer to the next free node.
};

/**
 * @brief Fixed-capacity, single-threaded object pool with O(1) acquire/release.
 *
 * Allocates all nodes upfront in a single contiguous block at construction time.
 * Subsequent acquire() and release() calls manipulate a singly-linked free list
 * without touching the heap.
 *
 * ### Usage
 * ```cpp
 * ObjectPool<ImuMessage> pool{128};
 * ImuMessage* msg = pool.acquire(); // returns nullptr if pool is exhausted
 * if (msg) {
 *     // fill in msg ...
 *     pool.release(msg);
 * }
 * ```
 *
 * @tparam T The type of object managed by this pool. Must be default-constructible.
 *
 * @note Not thread-safe. Use separate pools per thread or add external locking.
 * @note Copy and move are deleted to prevent accidental transfer of the backing block.
 */
template<typename T>
class ObjectPool
{
public:

    /**
     * @brief Allocates and initialises a pool of @p poolSize objects.
     *
     * Builds the intrusive free list so that every node points to the next one.
     * Construction time is O(poolSize); all subsequent operations are O(1).
     *
     * @param poolSize Number of objects to pre-allocate. A value of 0 creates a
     *                 valid but permanently empty pool (every acquire() returns nullptr).
     */
    explicit ObjectPool(size_t poolSize);

    ObjectPool(const ObjectPool&)            = delete; ///< Non-copyable.
    ObjectPool(ObjectPool&&)                 = delete; ///< Non-movable.
    ObjectPool& operator=(const ObjectPool&) = delete; ///< Non-copy-assignable.
    ObjectPool& operator=(ObjectPool&&)      = delete; ///< Non-move-assignable.

    /**
     * @brief Removes a node from the free list and returns a pointer to its data.
     *
     * Complexity: O(1), 0 heap allocations.
     *
     * @return Pointer to a default-constructed @c T, or @c nullptr if the pool
     *         is exhausted.
     */
    T* acquire();

    /**
     * @brief Returns a previously acquired object back to the free list.
     *
     * The node is recovered via a @c reinterpret_cast — this is safe because
     * @c PoolNode<T> begins with the @c data member and both share the same
     * aligned address (guaranteed by the struct layout).
     *
     * Complexity: O(1), 0 heap allocations.
     *
     * @param item Pointer originally returned by acquire(). Passing @c nullptr
     *             is a no-op. Passing a pointer not owned by this pool is
     *             undefined behaviour.
     */
    void release(T* item);

private:

    size_t m_poolSize;

    /// Single contiguous block of PoolNode<T> objects, managed by RAII.
    std::unique_ptr<PoolNode<T>[]> m_memoryBlock;

    /// Head of the singly-linked free list; nullptr when the pool is exhausted.
    PoolNode<T>* m_freeListHead{nullptr};
};

template<typename T>
ObjectPool<T>::ObjectPool(size_t poolSize) :
        m_poolSize(poolSize),
        m_memoryBlock(std::make_unique<PoolNode<T>[]>(poolSize))
{
    if (m_poolSize == 0)
    {
        return;
    }

    for (size_t i = 0; i < m_poolSize - 1; ++i)
    {
        m_memoryBlock[i].next = &m_memoryBlock[i + 1];
    }

    m_memoryBlock[m_poolSize - 1].next = nullptr;
    m_freeListHead = &m_memoryBlock[0];
}

template<typename T>
T* ObjectPool<T>::acquire()
{
    if (!m_freeListHead)
    {
        return nullptr;
    }

    PoolNode<T>* node = m_freeListHead;
    m_freeListHead    = m_freeListHead->next;

    return &(node->data);
}

template<typename T>
void ObjectPool<T>::release(T* item)
{
    if (!item)
    {
        return;
    }

    // PoolNode<T>::data is the first member, so the node address equals the data address.
    PoolNode<T>* node = reinterpret_cast<PoolNode<T>*>(item);

    node->next     = m_freeListHead;
    m_freeListHead = node;
}

} // namespace edge_sync::core

#endif // OBJECT_POOL_HPP
