#ifndef EPI_COMPONENT_GROUP_HPP
#define EPI_COMPONENT_GROUP_HPP
#include "entity.hpp"
#include "group.hpp"
#include "utils/map_key_intersection.hpp"
#include <set>

namespace epi {

template <class Enum>
class ComponentGroup : public Group<Enum> {
    typedef typename Group<Enum>::HashSize HashSize;

    std::unordered_map<EntityID, size_t> m_entityID_to_index_map;
    std::vector<EntityID> m_entityID_list;

    ComponentGroup(std::span<HashSize> types_stored, std::span<Enum> variables)
        : Group<Enum>(types_stored, variables) {}

    template <class... Types, class IntSeqT, IntSeqT... Ints>
    void m_update_id_sequenced(std::vector<Enum> identifiers,
                               std::function<void(EntityID, Types...)> f,
                               std::integer_sequence<IntSeqT, Ints...> int_seq);
    template <class... Types>
    void m_update_id(std::vector<Enum> identifiers,
                     std::function<void(EntityID, Types...)> f) 
    {
        m_update_id_sequenced(identifiers, f, std::index_sequence_for<Types...>{});
    }

public:
    const std::unordered_map<EntityID, size_t>& getEntityIDToIndexMap() const {
        return m_entityID_to_index_map;
    }
    typedef std::unique_ptr<ComponentGroup> pointer;
    template <class T, class... Rest>
    void push_back(EntityID entityid, T v, Rest... vals) {
        m_entityID_to_index_map.insert_or_assign(entityid, this->size());
        m_entityID_list.push_back(entityid);
        Group<Enum>::push_back(v, vals...);
    }
    void swap(size_t idx1, size_t idx2) {
        std::swap(m_entityID_to_index_map.at(m_entityID_list[idx1]),
                  m_entityID_to_index_map.at(m_entityID_list[idx2]));
        std::swap(m_entityID_list[idx1], m_entityID_list[idx2]);
        Group<Enum>::swap(idx1, idx2);
    }
    void erase(size_t index) {
        this->swap(index, this->size() - 1);
        m_entityID_to_index_map.erase(m_entityID_list.back());
        m_entityID_list.pop_back();
        Group<Enum>::pop_back(index);
    }
    void eraseByID(EntityID id) {
        auto index = m_entityID_to_index_map.at(id);
        erase(index);
    }
    void clear() {
        m_entityID_list.clear();
        m_entityID_to_index_map.clear();
        Group<Enum>::clear();
    }
    template <class T>
    inline std::optional<std::reference_wrapper<T>> getByID(Enum variable,
                                                            EntityID id) {
        auto itr = m_entityID_to_index_map.find(id);
        if (itr == m_entityID_to_index_map.end()) {
            return {};
        }
        return this->template get<T>(variable, itr->second);
    }
    template <class Iterable>
    void moveFoward(const Iterable& id_list) {
        size_t cursored_index = 0;
        for (auto id : id_list) {
            size_t other_index = m_entityID_to_index_map.at(id);
            swap(cursored_index++, other_index);
        }
    }
    template <class Func>
    void update_id(std::vector<Enum> identifiers, Func f) {
        m_update_id(identifiers, std::function(std::forward<Func>(f)));
    }

    ComponentGroup(const ComponentGroup&) = delete;
    ComponentGroup(ComponentGroup&&) = delete;
    ComponentGroup& operator=(const ComponentGroup&) = delete;
    ComponentGroup& operator=(ComponentGroup&&) = delete;
    std::vector<Buffer*> getBuffers(std::vector<Enum> identifiers) {
        return Group<Enum>::getBuffers(identifiers);
    }

    class Factory;
};
namespace helper {

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
template <class Enum1, class Enum2, class... Types>
void updateMatching(size_t intersection_size, ComponentGroup<Enum1>& g1,
                    ComponentGroup<Enum2>& g2, std::vector<Enum1> ids1,
                    std::vector<Enum2> ids2,
                    std::function<void(Types...)> update_func) {
    std::vector<std::vector<Buffer*>> buffers_2d = {g1.getBuffers(ids1),
                                                    g2.getBuffers(ids2)};
    std::vector<Buffer*> buffers = merge2DVectorsIntoOne(std::move(buffers_2d));
    size_t idx = 0;
    void* pointers[] = {
        (buffers[idx++]->getData<base_type<Types>>().data())...};
    idx = 0;
    for (size_t i = 0; i < intersection_size; i++) {
        update_func(
            {(reinterpret_cast<base_type<Types>*>(pointers[idx++]))[i]}...);
        idx = 0;
    }
}
}; // namespace helper
template <class Enum>
template <class... Types, class IntSeqT, IntSeqT... Ints>
void ComponentGroup<Enum>::m_update_id_sequenced(std::vector<Enum> identifiers,
                           std::function<void(EntityID, Types...)> update_func,
                           std::integer_sequence<IntSeqT, Ints...> int_seq) 
{
    std::vector<Buffer*> buffers = this->getBuffers(identifiers);

    void* pointers[] = {
        (buffers[Ints]->template getData<base_type<Types>>().data())...};
    for (size_t i = 0; i < this->size(); i++) {
        update_func( m_entityID_list[i],
            (reinterpret_cast<base_type<Types>*>(pointers[Ints]))[i]...);
    }
}
template <class Enum1, class Enum2, class Func>
void updateMatching(ComponentGroup<Enum1>& g1, ComponentGroup<Enum2>& g2,
                    std::vector<Enum1> ids1, std::vector<Enum2> ids2,
                    Func update_func) {
    auto intersection =
        IntersectMaps(g1.getEntityIDToIndexMap(), g2.getEntityIDToIndexMap());
    g1.moveFoward(intersection);
    g2.moveFoward(intersection);
    helper::updateMatching(intersection.size(), g1, g2, ids1, ids2,
                           std::function(std::forward<Func>(update_func)));
}

template <class Enum>
class ComponentGroup<Enum>::Factory {
    std::vector<ComponentGroup::HashSize> m_init_values;
    std::vector<Enum> m_identifiers;

public:
    template <class T>
    void add(Enum identifier) {
        m_init_values.push_back({typeid(T), sizeof(T)});
        m_identifiers.push_back(identifier);
    }
    ComponentGroup::pointer create() {
        std::unique_ptr<ComponentGroup> result{
            new ComponentGroup(m_init_values, m_identifiers)};
        return std::move(result);
    }
};

} // namespace epi
#endif // EPI_COMPONENT_GROUP_HPP
