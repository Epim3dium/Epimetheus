#ifndef EPI_COMPONENT_GROUP_HPP
#define EPI_COMPONENT_GROUP_HPP
#include "templates/group.hpp"
#include "templates/set.hpp"
#include <set>

namespace epi {

class ComponentGroup : private Group {

    typedef uint64_t ID_t;

    std::unordered_map<ID_t, size_t> m_entityID_to_index_map;
    std::vector<ID_t> m_entityID_list;
protected:
    typedef typename Group::HashSize HashSize;
    ComponentGroup(std::vector<HashSize> types_stored)
        : Group(types_stored) {}

    using Group::getBuffers;
public:
    using Group::size;
    using Group::get;

    bool contains(ID_t id) const {
        return m_entityID_to_index_map.contains(id);
    }
    const std::unordered_map<ID_t, size_t>& getEntityIDToIndexMap() const {
        return m_entityID_to_index_map;
    }
    typedef std::unique_ptr<ComponentGroup> pointer;
    template <class T, class... Rest>
    void push_back(ID_t entityid, T v, Rest... vals) {
        m_entityID_to_index_map.insert_or_assign(entityid, this->size());
        m_entityID_list.push_back(entityid);
        Group::push_back(v, vals...);
    }
    void swap(size_t idx1, size_t idx2) {
        std::swap(m_entityID_to_index_map.at(m_entityID_list[idx1]),
                  m_entityID_to_index_map.at(m_entityID_list[idx2]));
        std::swap(m_entityID_list[idx1], m_entityID_list[idx2]);
        Group::swap(idx1, idx2);
    }
    void erase(size_t index) {
        this->swap(index, this->size() - 1);
        m_entityID_to_index_map.erase(m_entityID_list.back());
        m_entityID_list.pop_back();
        Group::pop_back();
    }
    void eraseByID(ID_t id) {
        auto index = m_entityID_to_index_map.at(id);
        erase(index);
    }
    void clear() {
        m_entityID_list.clear();
        m_entityID_to_index_map.clear();
        Group::clear();
    }
    template <class T>
    inline std::optional<std::reference_wrapper<T>> getByID(ID_t id) {
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
    Form<Types...> mold();

    ComponentGroup(const ComponentGroup&) = delete;
    ComponentGroup(ComponentGroup&&) = delete;
    ComponentGroup& operator=(const ComponentGroup&) = delete;
    ComponentGroup& operator=(ComponentGroup&&) = delete;

    class Factory;
};
namespace helper {
template <class KeyType, class LeftValue>
epi::Set<KeyType> MapToSet(const std::unordered_map<KeyType, LeftValue>& map) {
    epi::Set<KeyType> result;
    for (auto& i : map) {
        result.insert(i.first);
    }
    return result;
}
template <class T>
static inline std::vector<T>
merge2DVectorsIntoOne(std::vector<std::vector<T>>&& vecs) {
    std::vector<T> result;
    result.reserve(16U);
    for (auto& v : vecs) {
        result.insert(result.end(), v.begin(), v.end());
    }
    return result;
}
};
template<class ...Types>
ComponentGroup::Form<Types...> ComponentGroup::mold() {
    return Form<Types...>(m_entityID_list, getBuffers<Types...>(), this->size());
}
template<class ...Types>
class ComponentGroup::Form : private Group::Form<Types...> {
    std::vector<ID_t> m_ids;
    Form (std::vector<ID_t> ids, std::vector<Buffer*> buffers, size_t size) : Group::Form<Types...>(buffers, size), m_ids(ids) {}

    using Group::Form<Types...>::m_bufs;
    template<class ...ArgT, size_t... Ints>
    typename std::enable_if<(std::is_same<base_type<ArgT>, Types>::value && ...), void>::type
    m_for_each(std::function<void(ID_t, ArgT...)>&& update_func, std::integer_sequence<size_t, Ints...> int_seq, size_t start, size_t end) {
        for(size_t i = start; i < std::min(size(), end); i++) {
            update_func(m_ids[i], (reinterpret_cast<Types*>(m_bufs[Ints]->template getData<Types>().data()))[i]...);
        }
    }
public:
    using Group::Form<Types...>::for_each;
    using Group::Form<Types...>::size;

    template<class Func>
    void for_each(Func&& update_func, size_t start = 0, size_t end = INFINITY) {
        m_for_each(std::function(std::forward<Func>(update_func)), std::index_sequence_for<Types...>{}, start, end);
    }

    friend ComponentGroup;
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

} // namespace epi
#endif // EPI_COMPONENT_GROUP_HPP
