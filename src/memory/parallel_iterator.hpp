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
protected:
    pointer m_ptr;
    std::ptrdiff_t m_pos = 0;
public:
    RawParallelIterator() : m_ptr({(Types*)nullptr}...){} 
    RawParallelIterator(Types*... pointers) : m_ptr({pointers}...) {} 

    RawParallelIterator(const IteratorType&) = default;
    RawParallelIterator<Types...>& operator=(const IteratorType&) = default;

    bool operator==(const IteratorType& other) const{
        return std::apply([&](auto&... myTuplePtr) 
            {
                return std::apply([&](auto&... OtherTuplePtr) 
                {
                    bool result = ((myTuplePtr + m_pos == OtherTuplePtr + other.m_pos) && ...);
                    return result;
                }, other.m_ptr);
            }, m_ptr);
    }
    bool operator!=(const IteratorType& other) const {
        return !operator==(other);
    }
    IteratorType& operator+=(const difference_type& movement) {
        m_pos += movement;
        return *this;
    }
    IteratorType& operator-=(const difference_type& movement) {
        m_pos -= movement;
        return *this;
    }
    IteratorType& operator++() {
        m_pos++;
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
        m_pos--;
        return *this;
    }
    IteratorType operator-(const difference_type& movement) const {
        auto temp = *this;
        temp.m_pos -= movement;
        return temp;
    }
    IteratorType operator+(const difference_type& movement) const {
        auto temp = *this;
        temp.m_pos += movement;
        return temp;
    }
    difference_type operator-(const IteratorType& other) const {
        return m_pos - other.m_pos;
    }

    reference operator*(){
        return std::apply([&](auto&... tuplePtr) 
            {
                reference result = {tuplePtr[m_pos]...};
                return result;
            }, m_ptr);
    }
    const_reference operator*() const {
        return std::apply([&](auto&... tuplePtr) 
            {
                const_reference result = {tuplePtr[m_pos]...};
                return result;
            }, m_ptr);
    }
    pointer operator->(){return m_ptr[m_pos];}
};
template <typename... Types>
class RawReverseParallelIterator : public RawParallelIterator<Types...> {
    typedef RawReverseParallelIterator<Types...> IteratorType;
    typedef RawParallelIterator<Types...> BaseType;
    typedef typename BaseType::difference_type difference_type;
public:
    RawReverseParallelIterator(Types*... pointers) : BaseType(pointers...) {} 
    RawReverseParallelIterator(const BaseType& rawIterator){this->m_ptr = rawIterator.getPtr();}
    RawReverseParallelIterator(const IteratorType& rawReverseIterator) = default;
    ~RawReverseParallelIterator(){}

    IteratorType&           operator=(const IteratorType& rawReverseIterator) = default;
    //IteratorType&           operator=(const BaseType& rawIterator){this->m_ptr = rawIterator.getPtr();return (*this);}
    //IteratorType&           operator=(Types*){this->setPtr(ptr);return (*this);}

    IteratorType& operator+=(const difference_type& movement) {
        this->m_pos -= movement;
        return *this;
    }
    IteratorType& operator-=(const difference_type& movement) {
        this->m_pos += movement;
        return *this;
    }
    IteratorType& operator++() {
        this->m_pos --;
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
        this->m_pos++;
        return *this;
    }
    IteratorType operator-(const difference_type& movement) const {
        auto temp = *this;
        temp -= movement;
        return temp;
    }
    IteratorType operator+(const difference_type& movement) const {
        auto temp = *this;
        temp += movement;
        return temp;
    }
    difference_type operator-(const IteratorType& other) const {
        return std::distance(std::get<0U>(this->m_ptr), std::get<0U>(other.m_ptr) );
    }
};
}
#endif
