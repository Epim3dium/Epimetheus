#ifndef EPI_PARALLEL_ITERATOR
#define EPI_PARALLEL_ITERATOR
#include <iterator>
//to iterate over multiple memory blocks .ex:
//int i[10]
//float f[10]
//RawPrallelIterator(i, f)
namespace epi {

template <typename... Types>
class RawParallelIterator {
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = std::tuple<Types...>;
    using difference_type = std::ptrdiff_t;
    using pointer = std::tuple<Types*...>;
    using reference = std::tuple<Types&...>;
    using const_reference = std::tuple<const Types&...>;
private:
    typedef RawParallelIterator<Types...> IteratorType;
    pointer m_ptr;
public:
    RawParallelIterator() : m_ptr({(Types*)nullptr}...){} 
    RawParallelIterator(Types*... pointers) : m_ptr({pointers}...) {} 

    RawParallelIterator(const IteratorType&) = default;
    RawParallelIterator<Types...>& operator=(const IteratorType&) = default;

    operator bool() const {
        if(std::get<0U>(m_ptr)) {
            return true;
        }
        else {
            return false;
        }
    }
    bool operator==(const IteratorType& other) const{
        return m_ptr == other.m_ptr;
    }
    bool operator!=(const IteratorType& other) const {
        return m_ptr != other.m_ptr;
    }
    IteratorType& operator+=(const difference_type& movement) {
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr += movement), ...);
            }, m_ptr);
        return *this;
    }
    IteratorType& operator-=(const difference_type& movement) {
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr -= movement), ...);
            }, m_ptr);
        return *this;
    }
    IteratorType& operator++() {
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr++), ...);
            }, m_ptr);
        return *this;
    }
    IteratorType operator++(int) {
        auto temp = *this;
        ++(*this);
        return temp;
    }
    IteratorType operator--(int) {
        auto temp = *this;
        --(*this);
        return temp;
    }
    IteratorType& operator--() {
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr--), ...);
            }, m_ptr);
        return *this;
    }
    IteratorType operator-(const difference_type& movement) const {
        auto temp = *this;
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr -= movement), ...);
            }, temp.m_ptr);
        return temp;
    }
    IteratorType operator+(const difference_type& movement) const {
        auto temp = *this;
        std::apply([&](auto&... tuplePtr) 
            {
                ((tuplePtr += movement), ...);
            }, temp.m_ptr);
        return temp;
    }
    difference_type operator-(const IteratorType& other) const {
        return std::distance(std::get<0U>(other.m_ptr), std::get<0U>(m_ptr) );
    }

    const_reference operator*() const {
        return std::apply([&](auto&... tuplePtr) 
            {
                const_reference result = {*tuplePtr...};
                return result;
            }, m_ptr);
    }
    reference operator*(){
        return std::apply([&](auto&... tuplePtr) 
            {
                reference result = {*tuplePtr...};
                return result;
            }, m_ptr);
    }
    pointer operator->(){return m_ptr;}
};

}
#endif
