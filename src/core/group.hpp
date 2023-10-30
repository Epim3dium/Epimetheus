#ifndef EPI_Group_HPP
#define EPI_Group_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <span>
#include <functional>

#include "buffer.hpp"

namespace epi {
#define EPI_GROUP_MAX_ELEMENT_COUNT 100000

template<typename T>
using base_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

template<class Enum>
class Group {
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    std::unordered_map<Enum, size_t> m_index_list;
    std::vector<Buffer> m_buffers;
    size_t m_var_count;
    size_t m_instances;

    Group(std::span<HashSize> types_stored, std::span<Enum> variables) : m_var_count(types_stored.size()) {
        size_t idx = 0;
        for(auto [hash, size] : types_stored) {
            m_index_list.insert({variables[idx], idx++});
            m_buffers.push_back(Buffer(hash, size, EPI_GROUP_MAX_ELEMENT_COUNT));
        }
    }

    void m_push_back(size_t buf_index) {}
    template<class T, class ...Rest>
    void m_push_back(size_t buf_index, T v, Rest ... vals) {
        m_buffers[buf_index++].push_back(v);
        m_push_back(buf_index, vals...);
    }

    size_t m_getBufferIdxOfType(Enum info) {
        auto itr = m_index_list.find(info);
        assert(itr != m_index_list.end());
        return itr->second;
    }


    template<class T>
    void m_getBufferPtrs(void** ptr, std::span<Enum> identifiers, size_t idx = 0) {
        auto buffer_idx = m_getBufferIdxOfType(identifiers[idx]);
        ptr[idx] = m_buffers[buffer_idx]. getDataPointer();
    }
    template<class T, class Next, class ...Types>
    void m_getBufferPtrs(void** ptr, std::span<Enum> identifiers, size_t idx = 0) {

        auto buffer_idx = m_getBufferIdxOfType(identifiers[idx]);
        ptr[idx] = m_buffers[buffer_idx].getDataPointer();

        m_getBufferPtrs<Next, Types...>(ptr, identifiers, idx + 1);
    }
    template<class ...Types>
    void m_update(std::span<Enum> identifiers, std::function<void(Types...)> update_func) {
        size_t type_count = sizeof...(Types);
        void* pointers[type_count];
        m_getBufferPtrs<Types...>(pointers, identifiers);
        size_t idx = 0;
        for(size_t i = 0; i < m_instances; i++) {
            update_func((( reinterpret_cast<base_type<Types>*>(pointers[idx++]))[i] )...);
            idx = 0;
        }
    }
public:
    size_t size() const {
        return m_instances;
    }
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
    T& get(Enum variable, size_t index) {
        auto itr = m_index_list.find(variable);
        assert(itr != m_index_list.end());
        return m_buffers[itr->second].
            template get<T>(index);
    }

    template<class Func>
    void update(std::vector<Enum> identifiers, Func&& f) {
        m_update(identifiers, std::function(std::forward<Func>(f)));
    }


    Group(const Group&) = delete;
    Group(Group&&) = delete;
    Group& operator=(const Group&) = delete;
    Group& operator=(Group&&) = delete;

    struct VariableGetter {
        Group<Enum>* group;
        std::vector<Enum> identifiers;
        std::vector<void*> getPointers() {

            std::vector<void*> ptrs;
            for(int i = 0; i < identifiers.size(); i++) {
                auto id = identifiers[i];
                auto buf_index = group->m_getBufferIdxOfType(id);
                ptrs.push_back(group->m_buffers[buf_index].getDataPointer());
            }
            return ptrs;
        }
    };
    class Factory;
};
    template<class ...Types>
    void updateMatchingHelper(std::function<void(Types...)> update_func, std::vector<void*> pointers, size_t iters) {
        size_t idx = 0;
        for(size_t i = 0; i < iters; i++) {
            update_func((( reinterpret_cast<base_type<Types>*>(pointers[idx++]))[i] )...);
            idx = 0;
        }
    }
    template<class ...EnumTypes, class Func>
    void updateMatching(Func f, typename Group<EnumTypes>::VariableGetter... getters) {
        std::vector<std::vector<void*>> pointers2d = {getters.getPointers()...};
        size_t iters = std::min({getters.group->size()...});
        std::vector<void*> pointers;
        for(auto& v : pointers2d) {
            pointers.insert(pointers.end(), v.begin(), v.end());
        }
        updateMatchingHelper(std::function(std::forward<Func>(f)), pointers, iters);
    }

template<class Enum>
class Group<Enum>::Factory {
    std::vector<Group::HashSize> m_init_values;
    std::vector<Enum> m_identifiers;
public:
    template<class T>
    void add(Enum identifier) {
        m_init_values.push_back({typeid(T), sizeof(T)});
        m_identifiers.push_back(identifier);
    }
    std::unique_ptr<Group> create() {
        std::unique_ptr<Group> result{ new Group(m_init_values, m_identifiers)};
        return std::move(result);
    }
};

};

#endif //EPI_Group_HPP
