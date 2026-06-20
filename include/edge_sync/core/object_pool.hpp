#ifndef OBJECT_POOL_HPP
#define OBJECT_POOL_HPP

#include <cstddef>
#include <memory>
#include <new>

namespace edge_sync::core
{
    
// Fallback macro: If the compiler doesn't support the C++17 hardware size, default to 64.
#ifdef __cpp_lib_hardware_interference_size
    using std::hardware_constructive_interference_size;
#else
    constexpr std::size_t hardware_constructive_interference_size = 64;
#endif

template<typename T>
struct alignas(hardware_constructive_interference_size) PoolNode 
{
    T data;
    PoolNode<T>* next{nullptr};
};

template<typename T>
class ObjectPool 
{
    public: 

        explicit ObjectPool(size_t poolSize);

        // delete copy and move semantics. prevents copying massive block of memory
        ObjectPool(const ObjectPool&) = delete;
        ObjectPool(ObjectPool&&) = delete;
        ObjectPool& operator=(const ObjectPool&) = delete;
        ObjectPool& operator=(ObjectPool&&) = delete;
        
        // Hot-path methods: Must execute in O(1) time with 0 allocations.
        T* acquire();
        void release(T* item);

    private:

        size_t m_poolSize;

        // The single, contiguous block of memory managed by RAII.
        std::unique_ptr<PoolNode<T>[]> m_memoryBlock;

        // Points to the first available node in the pool.
        PoolNode<T>* m_freeListHead{nullptr};
};

template<typename T>
ObjectPool<T>::ObjectPool(size_t poolSize) :
        m_poolSize(poolSize),
        m_memoryBlock(std::make_unique<PoolNode<T>[]>(poolSize))
{
    if(m_poolSize == 0)
    {
        return;
    }

    for(size_t i = 0; i < m_poolSize - 1; ++i)
    {
        m_memoryBlock[i].next = &m_memoryBlock[i + 1];
    }

    // last node must point to nullptr
    m_memoryBlock[m_poolSize - 1].next = nullptr;

    // set head pointer to point to first block of contigous memory
    m_freeListHead = &m_memoryBlock[0];
}

template<typename T>
T* ObjectPool<T>::acquire()
{
    if(!m_freeListHead)
    {
        return nullptr;
    }

    Pool<T>* node = m_freeListHead;
    m_freeListHead = m_freeListHead->next;

    return &(node->data);   
}

template<typename T>
void ObjectPool<T>::release(T* item)
{
    if(!item)
    {
        return;
    }

    // recover poolnode pointer from raw pointer
    PoolNode<T>* node = reinterpret_cast<PoolNode<T>*>(item);

    // push recovered node back onto head of free list O(1)
    node->next = m_freeListHead;
    m_freeListHead = node;
}
}

#endif /*OBJECT_POOL_HPP*/