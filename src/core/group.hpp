#ifndef EPI_Group_HPP
#define EPI_Group_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <span>
#include <functional>

#include "buffer.hpp"

namespace epi {
#define EPI_GROUP_MAX_ELEMENT_COUNT 10000000

template<typename T>
using base_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

template<class Enum>
class Group {
    std::unordered_map<Enum, size_t> m_buffer_index_list;
    std::vector<Buffer> m_buffers;
protected:
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    size_t m_var_count;
    size_t m_instances;

    Group(std::span<HashSize> types_stored, std::span<Enum> variables) : m_var_count(types_stored.size()) {
        size_t idx = 0;
        for(auto [hash, size] : types_stored) {
            m_buffer_index_list.insert({variables[idx], idx++});
            m_buffers.push_back(Buffer(hash, size, EPI_GROUP_MAX_ELEMENT_COUNT));
        }
    }
private:

    void m_push_back(size_t buf_index) {}
    template<class T, class ...Rest>
    void m_push_back(size_t buf_index, T v, Rest ... vals) {
        m_buffers[buf_index++].push_back(v);
        m_push_back(buf_index, vals...);
    }

    size_t m_getBufferIdxOfType(Enum info) {
        auto itr = m_buffer_index_list.find(info);
        assert(itr != m_buffer_index_list.end());
        return itr->second;
    }
public:
    typedef std::unique_ptr<Group> pointer;
    size_t size() const {
        return m_instances;
    }
    template<class T, class ...Rest>
    void push_back(T v, Rest ... vals) {
        assert(sizeof...(vals) + 1 == m_var_count && "tried to push uncomplete variable set");
        m_instances++;
        m_push_back(0, v, vals...);
    }
    void swap(size_t idx1, size_t idx2) {
        for(auto& b : m_buffers) {
            b.swap(idx1, idx2);
        }
    }
    void erase(size_t index) {
        for(auto& b : m_buffers) {
            b.swap(index, size() - 1);
            b.pop_back();
        }
        m_instances--;
    }
    void pop_back(size_t index) {
        for(auto& b : m_buffers) {
            b.pop_back();
        }
        m_instances--;
    }
    void clear() {
        m_instances = 0;
        for(auto& b : m_buffers)
            b.clear();
    }
    template<class T>
    T& get(Enum variable, size_t index) {
        auto itr = m_buffer_index_list.find(variable);
        assert(itr != m_buffer_index_list.end());
        return m_buffers[itr->second].
            template get<T>(index);
    }

    template<class Func>
    void update(std::vector<Enum> identifiers, Func&& f);


    Group(const Group&) = delete;
    Group(Group&&) = delete;
    Group& operator=(const Group&) = delete;
    Group& operator=(Group&&) = delete;

    std::vector<Buffer*> getBuffers(std::vector<Enum> identifiers) {
        std::vector<Buffer*> ptrs;
        for(int i = 0; i < identifiers.size(); i++) {
            auto id = identifiers[i];
            auto buf_index = m_getBufferIdxOfType(id);
            ptrs.push_back(&m_buffers[buf_index]);
        }
        return ptrs;
    }
    class Factory;
};

namespace helper {
    template<class T>
    static inline std::vector<T> merge2DVectorsIntoOne(std::vector<std::vector<T>>&& vecs) {
        std::vector<T> result;
        result.reserve(16U);
        for(auto& v : vecs) {
            result.insert(result.end(), v.begin(), v.end());
        }
        return result;
    }
    template<class ...EnumTypes, class ...Types>
    static inline void updateAny(std::function<void(Types...)>&& update_func,Group<EnumTypes>*... groups,
            std::vector<EnumTypes>... variables) 
    {
        std::vector<std::vector<Buffer*>> buffers_2d = {groups->getBuffers(variables)... };
        std::vector<Buffer*> buffers = merge2DVectorsIntoOne(std::move(buffers_2d));

        size_t idx = 0;
        void* pointers[] = {(buffers[idx++]->getData<base_type<Types>>().data())...};
        size_t iters = std::min({groups->size()...});
        idx = 0;
        for(size_t i = 0; i < iters; i++) {
            update_func(( reinterpret_cast<base_type<Types>*>(pointers[idx++]))[i]...);
            idx = 0;
        }
    }
}
template<class ...EnumTypes, class Func>
void updateAny(std::vector<EnumTypes>... identifiers, Func&& f, Group<EnumTypes>*... groups) {
    helper::updateAny<EnumTypes...>(std::function(std::forward<Func>(f)), groups..., identifiers...);
}

template<class Enum>
template<class Func>
void Group<Enum>::update(std::vector<Enum> identifiers, Func&& f) {
    updateAny<Enum>(identifiers, f, this);
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
