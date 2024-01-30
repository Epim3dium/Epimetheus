#ifndef EPI_GROUP_HPP
#define EPI_GROUP_HPP

#include <span>
#include <unordered_map>
#include <vector>
#include <tuple>
#include "parallel_iterator.hpp"
#include "entity.hpp"

namespace epi {

template <typename>
struct tuple_of_vectors;

template <typename... Ts>
struct tuple_of_vectors<std::tuple<Ts...>> {
  using type = std::tuple<std::vector<Ts>...>;
};

template <typename>
struct tuple_of_spans;

template <typename... Ts>
struct tuple_of_spans<std::tuple<Ts...>> {
    using type = std::tuple<std::span<Ts>...>;
};

template<typename What, typename ... Args>
struct is_present {
    static constexpr bool value {(std::is_same_v<What, Args> || ...)};
};

template <class ...Types>
class Clipping {
    using tuple_type = typename tuple_of_spans<std::tuple<Types...>>::type;
    tuple_type m_data_spans;
    const std::unordered_map<Entity, size_t>& m_entityToIndex;
public:
    using iterator = RawParallelIterator<Types...>;
    using const_iterator = RawParallelIterator<const Types...>;
    Clipping(const std::unordered_map<Entity, size_t>& entityToIndexMap, std::vector<Types>& ...vectors)
        : m_data_spans({vectors.data(), vectors.size()}...),
        m_entityToIndex(entityToIndexMap)
    {}
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
        return &std::get<std::span<CompTy>>(m_data_spans)[index];
        
    }
    iterator begin() {
        return std::apply([&](auto&... spans) {
                return iterator(spans.data()...);
            }, m_data_spans); 
    }
    iterator end() {
        return std::apply([&](auto&... spans) {
                return iterator(spans.data() + spans.size()...);
            }, m_data_spans); 
    }
    const_iterator cbegin() const {
        return std::apply([&](auto&... spans) {
                return const_iterator(spans.data()...);
            }, m_data_spans); 
    }
    const_iterator cend() const {
        return std::apply([&](auto&... spans) {
                return const_iterator(spans.data() + spans.size()...);
            }, m_data_spans); 
    }
};

 

template <class ...Types>
class Group {
    using tuple_type = typename tuple_of_vectors<std::tuple<Types...>>::type;
    tuple_type m_data;
    std::unordered_map<Entity, size_t> m_entityToIndex;
    size_t m_size = 0;

    void m_pop_back() {
        std::apply([&](auto&... tupleVecs) 
            {
                (tupleVecs.pop_back(), ...);
            }, m_data);
        m_size--;
    }
    void m_swap_elements(size_t idx1, size_t idx2) {
        std::apply([&](auto&... tupleVecs) 
            {
                (std::swap(tupleVecs[idx1], tupleVecs[idx2]), ...);
            }, m_data);
    }
public:
    using iterator = RawParallelIterator<Types...>;
    using const_iterator = RawParallelIterator<const Types...>;
    
    size_t size() const {
        return m_size;
    }
    iterator begin() {
        return std::apply([&](auto&... vecs) {
                return iterator(vecs.data()...);
            }, m_data); 
    }
    iterator end() {
        return std::apply([&](auto&... ves) {
                return iterator(ves.data() + ves.size()...);
            }, m_data); 
    }
    const_iterator cbegin() const {
        return std::apply([&](auto&... vecs) {
                return const_iterator(vecs.data()...);
            }, m_data); 
    }
    const_iterator cend() const {
        return std::apply([&](auto&... vecs) {
                return const_iterator(vecs.data() + vecs.size()...);
            }, m_data); 
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
        return true;
    }
    void push_back(Entity entity, Types... args) {
        std::apply([&](auto&... tupleVecs) 
            {
                (tupleVecs.push_back(args), ...);
            }, m_data);
        m_entityToIndex[entity] = m_size;
        m_size++;
    }
    template<class Ty>
    struct ClippingHelper {
        using type = std::vector<Ty>;
    };
    template<class ...ExtractedTy>
    Clipping<ExtractedTy...> createClipping() 
    {
        return Clipping<ExtractedTy...>(m_entityToIndex, std::get<std::vector<ExtractedTy>>(m_data)...);
    }
};

}

#endif
