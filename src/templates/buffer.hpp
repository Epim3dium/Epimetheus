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
        std::memcpy(ptr, tail, m_elem_size);
    }
public:
    template<class T>
    inline std::span<T> getData() const {
        assert(typeid(T).hash_code() == m_type_hash);
        return std::span<T>(reinterpret_cast<T*>(mem_block), m_elem_count );
    }
    inline void* getPtr() const {
        return mem_block;
    }
    inline void clear() {
        tail = mem_block;
        m_elem_count = 0;
    }
    template<class T>
    inline void push_back(T val) {
        assert(typeid(T).hash_code() == m_type_hash);
        assert(m_elem_count < m_max_elem_count);
        auto mem = allocate(sizeof(T));
        new (mem)T(val);
    }
    inline void swap(size_t idx1, size_t idx2) {
        auto head =(char*)mem_block;
        auto ptr1 = (head + idx1 * m_elem_size);
        auto ptr2 = (head + idx2 * m_elem_size);

        for (size_t i = 0; i < m_elem_size; i++) {
            char tmp = ptr1[i];
            ptr1[i] = ptr2[i];
            ptr2[i] = tmp;
        }

    }
    inline void pop_back() {
        assert(tail != mem_block);
        char* last = static_cast<char*>(tail);
        last -= m_elem_size;
        m_elem_count--;
        tail = last;
    }
    template<class T>
    inline T& get(size_t index) {
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
