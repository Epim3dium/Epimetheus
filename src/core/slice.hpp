#ifndef EPI_SLICE_HPP
#define EPI_SLICE_HPP
#include "entity.hpp"
#include "memory/parallel_iterator.hpp"
#include <span>
#include <tuple>
#include <unordered_map>
namespace epi {
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


template<typename First, typename... Ts>
struct isFirstEntity {
    static constexpr bool value = std::is_same<First, Entity>::value;
};

template <class ...Types>
class Slice {
    using tuple_type = typename tuple_of_spans<std::tuple<Types...>>::type;
    tuple_type m_data_spans;
    const std::unordered_map<Entity, size_t>& m_entityToIndex;
public:
    using iterator = RawParallelIterator<Types...>;
    using const_iterator = RawParallelIterator<const Types...>;
    using reverse_iterator = RawReverseParallelIterator<Types...>;
    using const_reverse_iterator = RawReverseParallelIterator<const Types...>;
    Slice(const std::unordered_map<Entity, size_t>& entityToIndexMap, std::vector<Types>& ...vectors)
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
    size_t size() const {
        return m_entityToIndex.size();
    }
    typename iterator::reference operator[](size_t index) {
        return *(std::next(this->begin(), index));
    }
    bool contains(Entity e) const {
        return m_entityToIndex.contains(e);
    }
    // typename iterator::const_reference operator[](size_t index) const {
    //     return *(begin() + index);
    // }
    template<class CompTy>
    std::optional<CompTy*> get(Entity entity) {
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
    reverse_iterator rbegin() {
        return std::apply([&](auto&... spans) {
                return reverse_iterator(spans.data() + spans.size() - 1 ...);
            }, m_data_spans); 
    }
    reverse_iterator rend() {
        return std::apply([&](auto&... spans) {
                return reverse_iterator(spans.data() - 1 ...);
            }, m_data_spans); 
    }
    const_reverse_iterator crbegin() const {
        return std::apply([&](auto&... spans) {
                return const_reverse_iterator(spans.data() + spans.size() - 1 ...);
            }, m_data_spans); 
    }
    const_reverse_iterator crend() const {
        return std::apply([&](auto&... spans) {
                return const_reverse_iterator(spans.data() - 1 ...);
            }, m_data_spans); 
    }
};
template<class ...Types>
struct OwnerSlice : Slice<Entity, Types...> {
    
    OwnerSlice(const std::unordered_map<Entity, size_t>& entityToIndexMap, std::vector<Entity>& owner, std::vector<Types>& ...vectors)
        : Slice<Entity, Types...>(entityToIndexMap, owner, vectors...) {}
};

}
#endif
