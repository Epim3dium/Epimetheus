#ifndef EPI_SYSTEM_HPP
#define EPI_SYSTEM_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <functional>
#include "memory/allocator.hpp"

namespace epi {
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
    inline void* getData() const {
        return mem_block;
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
        if(!(index < m_elem_count && "tried accessing element out of range")) {
            std::cerr << index << "\t" << m_elem_count << "\n";
            assert(false);
        }
        return static_cast<T*>(mem_block)[index];
    }
    Buffer(const std::type_info& type_hash, size_t elem_size, size_t elem_count) : m_type_hash(type_hash.hash_code()), m_elem_size(elem_size), m_max_elem_count(elem_count)
    {
        mem_block = malloc(m_elem_size * m_max_elem_count);
        tail = mem_block;
    }
};

#define EPI_MAX_ELEMENT_COUNT 100000000
class SystemFactory;
class System {
    typedef size_t hash_type;
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    std::unordered_multimap<hash_type, size_t> m_index_list;
    std::vector<Buffer> m_buffers;
    std::vector<bool> m_occupied_buffers;
    size_t m_var_count;
    size_t m_instances;

    System(std::vector<HashSize> types_stored) : m_var_count(types_stored.size()), m_occupied_buffers(types_stored.size(), false) {
        size_t idx = 0;
        for(auto [hash, size] : types_stored) {
            m_index_list.insert({hash.hash_code(), idx++});
            m_buffers.push_back(Buffer(hash, size, EPI_MAX_ELEMENT_COUNT));
        }
    }

    void m_push_back(size_t buf_index) {}
    template<class T, class ...Rest>
    void m_push_back(size_t buf_index, T v, Rest ... vals) {
        m_buffers[buf_index++].push_back(v);
        m_push_back(buf_index, vals...);
    }

    size_t m_getBufferIdxOfType(const std::type_info& info) {
        auto [begin, end] = m_index_list.equal_range(info.hash_code());
        while(begin != end && m_occupied_buffers[begin->second] == true) {
            begin++;
        }
        assert(begin != end);
        return begin->second;
    }
    template<class T>
    void m_getBufferPtrs(void** ptr, size_t idx = 0) {
        auto buffer_idx = m_getBufferIdxOfType(typeid(T));
        ptr[idx] = m_buffers[buffer_idx].getData();
    }
    template<class T, class Next, class ...Types>
    void m_getBufferPtrs(void** ptr, size_t idx = 0) {

        auto buffer_idx = m_getBufferIdxOfType(typeid(T));
        ptr[idx] = m_buffers[buffer_idx].getData();

        m_occupied_buffers[buffer_idx] = true;
        m_getBufferPtrs<Next, Types...>(ptr, idx + 1);
        m_occupied_buffers[buffer_idx] = false;
    }
    template<class ...Types>
    void m_update(std::function<void(Types...)> update_func) {
        size_t type_count = sizeof...(Types);
        void* pointers[type_count];
        m_getBufferPtrs<Types...>(pointers);
        size_t idx = 0;
        for(size_t i = 0; i < m_instances; i++) {
            update_func((( reinterpret_cast<typename std::remove_reference<Types>::type*>(pointers[idx++]))[i] )...);
            idx = 0;
        }
    }
public:
    template<class T, class ...Rest>
    void push_back(T v, Rest ... vals) {
        assert(sizeof...(vals) + 1 == m_var_count && "tried to push uncomplete variable set");
        m_instances++;
        m_push_back(0, v, vals...);
    }
    void clear() {
        m_instances = 0;
        for(auto& b : m_buffers)
            b.clear();
    }
    template<class T>
    T& get(size_t index) {
        auto itr = m_index_list.find(typeid(T).hash_code());
        assert(itr != m_index_list.end());
        return m_buffers[itr->second].get<T>(index);
    }

    template<class Func>
    void update(Func&& f) {
        m_update(std::function(std::forward<Func>(f)));
    }

    System(const System&) = delete;
    System(System&&) = delete;
    System& operator=(const System&) = delete;
    System& operator=(System&&) = delete;

    friend SystemFactory;
};
class SystemFactory {
    std::vector<System::HashSize> m_init_values;

    template<class T>
    void eval() {
        m_init_values.push_back({typeid(T), sizeof(T)});
    }
    template<class T, class Next, class ...R>
    void eval() {
        m_init_values.push_back({typeid(T), sizeof(T)});
        eval<Next, R...>();
    }
public:
    template<class ...Types>
    void add() {
        eval<Types...>();
    }
    std::unique_ptr<System> create() {
        std::unique_ptr<System> result{ new System(m_init_values)};
        return std::move(result);
    }
};
};

#endif //EPI_SYSTEM_HPP
