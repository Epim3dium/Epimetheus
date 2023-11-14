#ifndef EPI_Group_HPP
#define EPI_Group_HPP

#include <functional>
#include <iostream>
#include <span>
#include <unordered_map>
#include <vector>

#include "buffer.hpp"

namespace epi {
#define EPI_GROUP_MAX_ELEMENT_COUNT 10000000

template <typename T>
using base_type =
    typename std::remove_cv<typename std::remove_reference<T>::type>::type;

class Group {
    typedef size_t hash_t ;
    std::unordered_map<hash_t, size_t> m_buffer_index_list;
    std::vector<Buffer> m_buffers;

protected:
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    size_t m_var_count;
    size_t m_instances;

    Group(std::span<HashSize> types_stored)
        : m_var_count(types_stored.size()) {
        size_t idx = 0;
        for (auto [hash, size] : types_stored) {
            m_buffer_index_list.insert({hash.hash_code(), idx++});
            m_buffers.push_back(
                Buffer(hash, size, EPI_GROUP_MAX_ELEMENT_COUNT));
        }
    }

private:
    void m_push_back(size_t buf_index) {}
    template <class T, class... Rest>
    void m_push_back(size_t buf_index, T v, Rest... vals) {
        m_buffers[buf_index++].push_back(v);
        m_push_back(buf_index, vals...);
    }

    template<class T>
    size_t m_getBufferIdxOfType() {
        auto itr = m_buffer_index_list.find(typeid(T).hash_code());
        assert(itr != m_buffer_index_list.end());
        return itr->second;
    }

protected:
    template<class ...Types>
    std::vector<Buffer*> getBuffers() {
        std::vector<size_t> indexes = {m_getBufferIdxOfType<Types>()...};
        std::vector<Buffer*> ptrs;
        for(auto i : indexes) {
            ptrs.push_back(&m_buffers[i]);
        }
        return ptrs;
    }
public:
    typedef std::unique_ptr<Group> pointer;
    size_t size() const { return m_instances; }
    template <class T, class... Rest>
    void push_back(T v, Rest... vals) {
        assert(sizeof...(vals) + 1 == m_var_count &&
               "tried to push uncomplete variable set");
        m_instances++;
        m_push_back(0, v, vals...);
    }
    void swap(size_t idx1, size_t idx2) {
        for (auto& b : m_buffers) {
            b.swap(idx1, idx2);
        }
    }
    void erase(size_t index) {
        for (auto& b : m_buffers) {
            b.swap(index, size() - 1);
            b.pop_back();
        }
        m_instances--;
    }
    void pop_back() {
        for (auto& b : m_buffers) {
            b.pop_back();
        }
        m_instances--;
    }
    void clear() {
        m_instances = 0;
        for (auto& b : m_buffers)
            b.clear();
    }
    template <class T>
    T& get(size_t index) {
        auto idx = m_getBufferIdxOfType<T>();
        return m_buffers[idx].template get<T>(index);
    }

    Group(const Group&) = delete;
    Group(Group&&) = delete;
    Group& operator=(const Group&) = delete;
    Group& operator=(Group&&) = delete;

    virtual ~Group() {}

    template<class ...Types>
    class Form;

    template<class ...Types>
    Form<Types...> mold();

    class Factory;
};

template<class ...Types>
Group::Form<Types...> Group::mold() {
    return Form<Types...>(getBuffers<Types...>(), this->size());
}
template<class ...Types>
class Group::Form {
protected:
    std::vector<Buffer*> m_bufs;
    Form (std::vector<Buffer*> buffers, size_t size) : m_bufs(buffers), m_size(size) {}
private:
    const size_t m_size;

    template<class ...ArgT, size_t... Ints>
    typename std::enable_if<(std::is_same<base_type<ArgT>, Types>::value && ...), void>::type
    m_for_each(std::function<void(ArgT...)>&& update_func, std::integer_sequence<size_t, Ints...> int_seq, size_t start, size_t end) {
        for(size_t i = start; i < std::min(m_size, end); i++) {
            update_func((reinterpret_cast<Types*>(m_bufs[Ints]->template getData<Types>().data()))[i]...);
        }
    }
public:
    size_t size() const {
        return m_size;
    }
    template<class Func>
    void for_each(Func&& update_func, size_t start = 0, size_t end = INFINITY) {
        m_for_each(std::function(std::forward<Func>(update_func)), std::index_sequence_for<Types...>{}, start, end);
    }
    friend Group;
    
};

class Group::Factory {
    std::vector<Group::HashSize> m_init_values;

public:
    template <class T>
    Group::Factory& add() {
        m_init_values.push_back({typeid(T), sizeof(T)});
        return *this;
    }
    std::unique_ptr<Group> create() {
        std::unique_ptr<Group> result{new Group(m_init_values)};
        return std::move(result);
    }
};

}; // namespace epi

#endif // EPI_Group_HPP
