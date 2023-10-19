#ifndef EPI_SYSTEM_HPP
#define EPI_SYSTEM_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <functional>
#include "memory/allocator.hpp"

namespace epi {

template<class T, class ...Types>
struct advance_pointers {
public:
    advance_pointers(void** t) {
        (advance_pointers<T>(t));
        (advance_pointers<Types...>(&t[1]));
    }
};
template<class T>
struct advance_pointers<T> {
public:
    advance_pointers(void** t) {
        ((char*&)t[0]) += sizeof(T);
    }
};

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
};

namespace epi {

#define EPI_MAX_ELEMENT_COUNT 16384
class SystemFactory;
class System {
    typedef size_t hash_type;
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    std::unordered_map<hash_type, size_t> m_index_list;
    size_t m_var_count;
    size_t m_instances;

    System(std::vector<HashSize> types_stored) : m_var_count(types_stored.size()) {
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
public:
    std::vector<Buffer> m_buffers;
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

    template<class ...T>
    inline void updatePointers(void** t) {
        (advance_pointers<T...>(t));
    }
    template<class T>
    T& expand(void**& t) {
        auto ptr = t[0];
        t = &t[1];
        return *(T*)ptr;
    }
    template<class Func>
    void update(Func&& f) {
        update((std::forward<Func>(f)));
    }
    template<class ...Types>
    void update(void (*update_func)(Types&...)) {
        //then get tuple
        size_t type_count = sizeof...(Types);
        void* pointers[type_count];
        //std::tuple<Types*...> list_of_pointers;
        setPointers<Types...>(0, 0, m_buffers.data(), pointers);
        for(size_t i = 0; i < m_instances; i++) {
            auto tmp = pointers;
            update_func((expand<Types>(tmp))...);

            updatePointers<Types...>(pointers);
        }
    }
    template<class T>
    size_t setPointers(size_t idx, size_t buffer_idx, Buffer* buf_array, void** data) {
        T* var = &buf_array[buffer_idx].get<T>(idx);
        data[buffer_idx] = (void*)var;
        return buffer_idx + 1;
    }
    template<class T, class Tnext, class ...Trest>
    void setPointers(size_t idx, size_t buffer_idx, Buffer* buf_array, void** data) {
        buffer_idx = setPointers<T>(idx, buffer_idx, buf_array, data);
        setPointers<Tnext, Trest...>(idx, buffer_idx, buf_array, data);
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
