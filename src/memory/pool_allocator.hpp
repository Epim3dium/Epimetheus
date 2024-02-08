//from: https://github.com/mtrebi/memory-allocators/blob/master/src/PoolAllocator.cpp
#include "allocator.hpp"
#include "templates/stack_linked_list.hpp"

#ifndef EPI_POOL_ALLOCATOR_H
#define EPI_POOL_ALLOCATOR_H

namespace epi {

class PoolAllocator : public Allocator {
private:
    struct  FreeHeader{
    };
    using Node = StackLinkedList<FreeHeader>::Node;
    StackLinkedList<FreeHeader> m_freeList;

    void * m_start_ptr = nullptr;
    std::size_t m_chunkSize;
public:
    PoolAllocator(const std::size_t totalSize, const std::size_t chunkSize);

    inline void* getStartPtr() const { return m_start_ptr; }
    inline size_t getTotalSize() const { return m_totalSize; }

    virtual ~PoolAllocator();

    virtual void* Allocate(const std::size_t size, const std::size_t alignment = 0) override;

    virtual void Free(void* ptr) override;

    virtual void Init() override;

    virtual void Reset();
private:
    PoolAllocator(PoolAllocator &poolAllocator);

};
}
#endif
