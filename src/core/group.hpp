#ifndef EPI_GROUP_HPP
#define EPI_GROUP_HPP

#include <span>
#include <unordered_map>
#include <vector>
#include <tuple>
#include "memory/parallel_iterator.hpp"
#include "entity.hpp"
#include "slice.hpp"

namespace epi {

template <typename>
struct tuple_of_vectors;

template <typename... Ts>
struct tuple_of_vectors<std::tuple<Ts...>> {
  using type = std::tuple<std::vector<Ts>...>;
};

template <class ...Types>
class Group {
    using tuple_type = typename tuple_of_vectors<std::tuple<Types...>>::type;
    tuple_type m_data;
    std::vector<Entity> m_entity_ids;
    std::unordered_map<Entity, size_t> m_entityToIndex;
    size_t m_size = 0;

    void m_pop_back() {
        std::apply([&](auto&... tupleVecs) 
            {
                (tupleVecs.pop_back(), ...);
            }, m_data);
        m_entity_ids.pop_back();
        m_size--;
    }
    void m_swap_elements(size_t idx1, size_t idx2) {
        std::apply([&](auto&... tupleVecs) 
            {
                (std::swap(tupleVecs[idx1], tupleVecs[idx2]), ...);
            }, m_data);
        m_entityToIndex.at(m_entity_ids[idx1]) = idx2;
        m_entityToIndex.at(m_entity_ids[idx2]) = idx1;
        std::swap(m_entity_ids[idx1], m_entity_ids[idx2]);
    }
public:
    std::tuple<Types...> default_values;
    using iterator = RawParallelIterator<Entity, Types...>;
    using const_iterator = RawParallelIterator<const Entity, const Types...>;
    using reverse_iterator = RawReverseParallelIterator<Entity, Types...>;
    using const_reverse_iterator = RawReverseParallelIterator<const Entity, const Types...>;
    
    size_t size() const {
        return m_size;
    }
    iterator begin() {
        return std::apply([&](auto&... vecs) {
                return iterator(m_entity_ids.data(), vecs.data()...);
            }, m_data); 
    }
    iterator end() {
        return std::apply([&](auto&... ves) {
                return iterator(m_entity_ids.data() + m_entity_ids.size(), ves.data() + ves.size()...);
            }, m_data); 
    }
    const_iterator cbegin() const {
        return std::apply([&](auto&... vecs) {
                return const_iterator(m_entity_ids.data(), vecs.data()...);
            }, m_data); 
    }
    const_iterator cend() const {
        return std::apply([&](auto&... vecs) {
                return const_iterator(m_entity_ids.data() + m_entity_ids.size(), vecs.data() + vecs.size()...);
            }, m_data); 
    }
    
    reverse_iterator rbegin() {
        return std::apply([&](auto&... vecs) {
                return reverse_iterator(m_entity_ids.data() + m_entity_ids.size() - 1, vecs.data() + vecs.size() - 1 ...);
            }, m_data); 
    }
    reverse_iterator rend() {
        return std::apply([&](auto&... ves) {
                return reverse_iterator(m_entity_ids.data() - 1, ves.data() - 1 ...);
            }, m_data); 
    }
    const_reverse_iterator crbegin() const {
        return std::apply([&](auto&... vecs) {
                return const_reverse_iterator(m_entity_ids.data() + m_entity_ids.size() - 1, vecs.data() + vecs.size() - 1 ...);
            }, m_data); 
    }
    const_reverse_iterator crend() const {
        return std::apply([&](auto&... vecs) {
                return const_reverse_iterator(m_entity_ids.data() - 1, vecs.data() - 1 ...);
            }, m_data); 
    }


    bool contains(Entity entity) {
        auto itr = m_entityToIndex.find(entity);
        if(itr != m_entityToIndex.end()) {
            return true;
        }
        return false;
    }
    
    std::optional<size_t> 
    getIndex(Entity entity) const {
        auto itr = m_entityToIndex.find(entity);
        if(itr != m_entityToIndex.end()) {
            return itr->second;
        }
        return {};
    }
    template<class CompTy>
    std::optional<CompTy*> getComponent(Entity entity) {
        static_assert(is_present<CompTy, Types...>::value);
        auto index_found = getIndex(entity);
        if(!index_found.has_value()) {
            return {};
        }
        size_t index = index_found.value();
        return &std::get<std::vector<CompTy>>(m_data)[index];
        
    }
    template<class CompTy>
    std::optional<const CompTy *> cgetComponent(Entity entity) const {
        static_assert(is_present<CompTy, Types...>::value);
        auto index_found = getIndex(entity);
        if(!index_found.has_value()) {
            return {};
        }
        size_t index = index_found.value();
        return &std::get<std::vector<CompTy>>(m_data)[index];
        
    }
    std::tuple<Types&...> at(Entity entity) {
        auto index_found = getIndex(entity);
        if(!index_found.has_value()) {
            throw std::out_of_range("Group::at");
        }
        size_t index = index_found.value();
        std::tuple<Types&...> result = {std::get<std::vector<Types>>(m_data)[index]...};
        return result;
    }
    //return true if found and erased object, false if couldn't find entity
    bool erase(Entity entity) {
        auto index_found = getIndex(entity);
        if(!index_found.has_value()) {
            return false;
        }
        size_t index = index_found.value();
        m_swap_elements(index, m_size - 1);
        m_pop_back();
        m_entityToIndex.erase(entity);
        return true;
    }
    void push_back(Entity entity, Types... args){
        m_entity_ids.push_back(entity);
        std::apply([&](auto&... tupleVecs) 
            {
                (tupleVecs.push_back(args), ...);
            }, m_data);
        m_entityToIndex[entity] = m_size;
        m_size++;
    }
    template<class ...InitializedTypes>
    void push_back(Entity entity, InitializedTypes... args){
        m_entity_ids.push_back(entity);
        
        std::tuple<Types...> init_values = default_values;
        ((std::get<InitializedTypes>(init_values) = args), ...);
        
        std::apply([&](std::vector<Types>&... tupleVecs) 
            {
                (tupleVecs.push_back(std::get<Types>(init_values)), ...);
            }, m_data);
        m_entityToIndex[entity] = m_size;
        m_size++;
    }
    Slice<Types...> sliceAll() 
    {
        return Slice<Types...>(m_entityToIndex, std::get<std::vector<Types>>(m_data)...);
    }
    Slice<Entity, Types...> sliceAllOwner() 
    {
        return Slice<Entity, Types...>(m_entityToIndex, m_entity_ids, std::get<std::vector<Types>>(m_data)...);
    }
    template<class ...ExtractedTy>
    Slice<Entity, ExtractedTy...> sliceOwner() 
    {
        return Slice<Entity, ExtractedTy...>(m_entityToIndex, m_entity_ids, std::get<std::vector<ExtractedTy>>(m_data)...);
    }
    template<class ...ExtractedTy>
    Slice<ExtractedTy...> slice() 
    {
        return Slice<ExtractedTy...>(m_entityToIndex, std::get<std::vector<ExtractedTy>>(m_data)...);
    }
};
}

#endif
