#ifndef EPI_COMPONENT_GROUP_HPP
#define EPI_COMPONENT_GROUP_HPP
#include "templates/buffer.hpp"
#include "templates/set.hpp"
#include "scene/entity.hpp"
#include <set>
#include <unordered_map>

namespace epi {

#define EPI_GROUP_MAX_ELEMENT_COUNT 10000000

template <typename T>
using base_type =
    typename std::remove_cv<typename std::remove_reference<T>::type>::type;

class ComponentGroup {
    typedef size_t hash_t ;
    std::unordered_map<hash_t, size_t> m_buffer_index_list;
    std::vector<Buffer> m_buffers;

    std::unordered_map<Entity, size_t> m_entityID_to_index_map;
    Buffer m_entityID_list;
    struct HashSize {
        const std::type_info& hash;
        size_t size;
    };
    size_t m_var_count;
    size_t m_instances;
    ComponentGroup(std::vector<HashSize> types_stored)
        : m_var_count(types_stored.size()), m_entityID_list(typeid(Entity), sizeof(Entity), EPI_GROUP_MAX_ELEMENT_COUNT) 
    {
        size_t idx = 0;
        for (auto [hash, size] : types_stored) {
            m_buffer_index_list.insert({hash.hash_code(), idx++});
            m_buffers.push_back(
                Buffer(hash, size, EPI_GROUP_MAX_ELEMENT_COUNT));
        }
    }
    template<class T>
    std::optional<size_t> m_getBufferIdxOfType() {
        auto itr = m_buffer_index_list.find(typeid(T).hash_code());
        if(itr == m_buffer_index_list.end())
            return {};
        return itr->second;
    }
    template<class ...Types>
    std::vector<void*> getData() {
        std::vector<void*> result = {m_buffers[m_getBufferIdxOfType<Types>().value()].template getData<Types>().data()...};
        return result;
    }
    void m_push_back(size_t buf_index) {}
    template <class T, class... Rest>
    void m_push_back(size_t buf_index, T v, Rest... vals) {
        m_buffers[buf_index++].push_back(v);
        m_push_back(buf_index, vals...);
    }

public:
    typedef std::unique_ptr<ComponentGroup> pointer;
    size_t size() const { return m_instances; }
    template <class T>
    T* get(size_t index) {
        auto idx = m_getBufferIdxOfType<T>();
        if(!idx.has_value())
            return nullptr;
        return &m_buffers[idx.value()].template get<T>(index);
    }

    bool contains(Entity id) const {
        return m_entityID_to_index_map.contains(id);
    }
    const std::unordered_map<Entity, size_t>& getEntityIDToIndexMap() const {
        return m_entityID_to_index_map;
    }
    template <class T, class... Rest>
    void push_back(Entity entityid, T v, Rest... vals) {
        m_entityID_to_index_map.insert_or_assign(entityid, this->size());
        m_entityID_list.push_back(entityid);
        assert(sizeof...(vals) + 1 == m_var_count &&
               "tried to push uncomplete variable set");
        m_instances++;
        m_push_back(0, v, vals...);
    }
    void swap(size_t idx1, size_t idx2) {
        std::swap(m_entityID_to_index_map.at(m_entityID_list.getData<Entity>()[idx1]),
                  m_entityID_to_index_map.at(m_entityID_list.getData<Entity>()[idx2]));
        std::swap(m_entityID_list.getData<Entity>()[idx1], m_entityID_list.getData<Entity>()[idx2]);
        for (auto& b : m_buffers) {
            b.swap(idx1, idx2);
        }
    }
    void pop_back() {
        for (auto& b : m_buffers) {
            b.pop_back();
        }
        m_entityID_list.pop_back();
        m_instances--;
    }
    void erase(size_t index) {
        this->swap(index, this->size() - 1);
        m_entityID_to_index_map.erase(m_entityID_list.getData<Entity>().back());
        pop_back();
    }
    void eraseByID(Entity id) {
        auto index = m_entityID_to_index_map.at(id);
        erase(index);
    }
    void clear() {
        m_entityID_list.clear();
        m_entityID_to_index_map.clear();

        m_instances = 0;
        for (auto& b : m_buffers)
            b.clear();
    }
    template <class T>
    inline std::optional<std::reference_wrapper<T>> getByID(Entity id) {
        auto itr = m_entityID_to_index_map.find(id);
        if (itr == m_entityID_to_index_map.end()) {
            return {};
        }
        return this->template get<T>(itr->second);
    }
    template <class Iterable>
    void moveFoward(const Iterable& id_list) {
        size_t cursored_index = 0;
        for (auto id : id_list) {
            size_t other_index = m_entityID_to_index_map.at(id);
            swap(cursored_index++, other_index);
        }
    }
    template<class ...Types>
    class Form;

    template<class ...Types>
    Form<Types...> formulize();

    ComponentGroup(const ComponentGroup&) = delete;
    ComponentGroup(ComponentGroup&&) = delete;
    ComponentGroup& operator=(const ComponentGroup&) = delete;
    ComponentGroup& operator=(ComponentGroup&&) = delete;

    class Factory;
};
class ComponentGroup::Factory {
    std::vector<ComponentGroup::HashSize> m_init_values;

public:
    template <class T>
    ComponentGroup::Factory& add() {
        m_init_values.push_back({typeid(T), sizeof(T)});
        return *this;
    }
    ComponentGroup::pointer create() {
        std::unique_ptr<ComponentGroup> result{
            new ComponentGroup(m_init_values)};
        return std::move(result);
    }
};
template<class ...Types>
ComponentGroup::Form<Types...> ComponentGroup::formulize() {
    return Form<Types...>(m_entityID_list.getData<Entity>(), getData<Types...>());
}

template<class ...Types>
class ComponentGroup::Form {
    std::span<Entity> m_ids;
    std::vector<void*> m_data;
    Form (std::span<Entity> ids, std::vector<void*> data) : m_data(data), m_ids(ids) {}

    template<class FirstT, class ...ArgT, size_t... Ints>
    typename std::enable_if<(std::is_same<base_type<ArgT>, Types>::value && ... && std::is_same<base_type<FirstT>, Entity>::value), void>::type
    m_for_each(std::function<void(FirstT, ArgT...)>&& update_func, std::integer_sequence<size_t, Ints...> int_seq, size_t start, size_t end) {
        for(size_t i = start; i < std::min(size(), end); i++) {
            update_func(m_ids[i], (reinterpret_cast<Types*>(m_data[Ints]))[i]...);
        }
    }
    template<class ...ArgT, size_t... Ints>
    typename std::enable_if<(std::is_same<base_type<ArgT>, Types>::value && ...), void>::type
    m_for_each(std::function<void(ArgT...)>&& update_func, std::integer_sequence<size_t, Ints...> int_seq, size_t start, size_t end) {
        for(size_t i = start; i < std::min(size(), end); i++) {
            update_func((reinterpret_cast<Types*>(m_data[Ints]))[i]...);
        }
    }

    template<size_t... Ints>
    std::tuple<Entity, Types&...> m_getTuple(size_t index, std::integer_sequence<size_t, Ints...> int_seq) {
        assert(index < size());
        return {m_ids[index], (reinterpret_cast<Types*>(m_data[Ints]))[index]...};
    }
    std::tuple<Entity, Types&...> getTuple(size_t index) {
        return m_getTuple(index, std::index_sequence_for<Types...>{});
    }

    template<size_t... Ints>
    std::tuple<Entity, const Types&...> m_cgetTuple(size_t index, std::integer_sequence<size_t, Ints...> int_seq) const {
        assert(index < size());
        return {m_ids[index], (reinterpret_cast<Types*>(m_data[Ints]))[index]...};
    }
    std::tuple<Entity, const Types&...> cgetTuple(size_t index) const {
        return m_cgetTuple(index, std::index_sequence_for<Types...>{});
    }
public:
    size_t size() const {return m_ids.size(); }

    template<class Func>
    void forEach(Func&& update_func, size_t start = 0, size_t end = INFINITY) {
        m_for_each(std::function(std::forward<Func>(update_func)), std::index_sequence_for<Types...>{}, start, end);
    }

    struct Iterator;
    Iterator begin() { return Iterator({this, 0}); }
    Iterator end()   { return Iterator({this, size()}); }
    struct ConstIterator;
    ConstIterator begin() const { return ConstIterator({this, 0}); }
    ConstIterator end()   const { return ConstIterator({this, size()}); }

    friend ComponentGroup;
};

template<class ...Types>
class ComponentGroup::Form<Types...>::Iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using initializer       = std::pair<ComponentGroup::Form<Types...>*, size_t>;  // or also value_type*
    using reference         = std::tuple<Entity, Types&...>;  // or also value_type&
                                    
    initializer m_ptr;
public:
    Iterator(initializer ptr) : m_ptr(ptr) {}

    reference operator*() const { 
        return m_ptr.first->getTuple(m_ptr.second);
    }
    Iterator& operator++() { m_ptr.second++; return *this; }
    Iterator& operator--() { m_ptr.second--; return *this; }
    friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
    friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };
};

template<class ...Types>
class ComponentGroup::Form<Types...>::ConstIterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using initializer       = std::pair<const ComponentGroup::Form<Types...>*, size_t>; 
    using reference         = std::tuple<Entity, const Types&...>;
                                    
    initializer m_ptr;
public:
    ConstIterator(initializer ptr) : m_ptr(ptr) {}

    reference operator*() const { 
        return m_ptr.first->cgetTuple(m_ptr.second);
    }
    ConstIterator& operator++() { m_ptr.second++; return *this; }
    ConstIterator& operator--() { m_ptr.second--; return *this; }
    friend bool operator== (const ConstIterator& a, const ConstIterator& b) { return a.m_ptr == b.m_ptr; };
    friend bool operator!= (const ConstIterator& a, const ConstIterator& b) { return a.m_ptr != b.m_ptr; };
};

} // namespace epi
#endif // EPI_COMPONENT_GROUP_HPP
