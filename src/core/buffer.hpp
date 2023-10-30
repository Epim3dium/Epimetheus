#ifndef EPI_BUFFER_HPP
#define EPI_BUFFER_HPP
#include <iostream>
#include <vector>
#include <span>
#include <functional>


class Buffer {
    void* mem_block;
    //points to last free element
    void* tail;

    const size_t m_elem_size;
    const size_t m_max_elem_count;
    size_t m_elem_count = 0U;

    size_t m_type_hash;

    void* allocate(const size_t size) {
        //check bounds
        assert(m_elem_count < m_max_elem_count);
        assert(size == m_elem_size);

        auto result = tail;
        {
            char* ptr = static_cast<char*>(tail);
            ptr += m_elem_size;
            m_elem_count++;
            tail = ptr;
        }
        return result;
    }
    void swapAndFree(void* ptr) {
        //check bounds
        assert(tail != mem_block);
        {
            char* last = static_cast<char*>(tail);
            last -= m_elem_size;
            m_elem_count--;
            tail = last;
        }
        std::memcpy(tail, ptr, m_elem_size);
    }
public:
    inline void* getDataPointer() const {
        return mem_block;
    }
    template<class T>
    inline std::span<T> getData() const {
        assert(typeid(T).hash_code() == m_type_hash);
        return std::span<T>(reinterpret_cast<T*>(mem_block), m_elem_count );
    }
    void clear() {
        tail = mem_block;
        m_elem_count = 0;
    }
    template<class T>
    void push_back(T val) {
        assert(typeid(T).hash_code() == m_type_hash);
        assert(m_elem_count < m_max_elem_count);
        auto mem = allocate(sizeof(T));
        new (mem)T(val);
    }
    void erase(size_t index) {
        assert(index < m_elem_count);
        auto ptr =(char*)mem_block;
        swapAndFree(ptr + index * m_elem_size);
    }
    template<class T>
    T& get(size_t index) {
        assert(typeid(T).hash_code() == m_type_hash);
        assert(index < m_elem_count && "tried accessing element out of range");
        return static_cast<T*>(mem_block)[index];
    }
    Buffer(const std::type_info& type_hash, size_t elem_size, size_t elem_count) 
        : m_type_hash(type_hash.hash_code()), m_elem_size(elem_size), m_max_elem_count(elem_count)
    {
        mem_block = malloc(m_elem_size * m_max_elem_count);
        tail = mem_block;
    }
};

#endif //EPI_BUFFER_HPP
