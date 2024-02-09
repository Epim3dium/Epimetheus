#ifndef EPI_PRiMITIVE_WRAPPER_HPP
#define EPI_PRiMITIVE_WRAPPER_HPP
#include <type_traits>
namespace epi {
#define EPI_WRAP_TYPE(WrappedType, Name)                                       \
    struct Name : public WrappedType {                                         \
        Name& operator=(const WrappedType& v) {                                \
            WrappedType::operator=(v);                                         \
            return *this;                                                      \
        }                                                                      \
        Name& operator=(WrappedType&& v) {                                     \
            WrappedType::operator=(v);                                         \
            return *this;                                                      \
        }                                                                      \
        Name(const WrappedType& v) : WrappedType(v) {}                         \
        Name(WrappedType&& v) : WrappedType(v) {}                              \
                                                                               \
        Name(const Name& v) = default;                                         \
        Name(Name&& v) = default;                                              \
        Name& operator=(const Name& v) = default;                              \
        Name& operator=(Name&& v) = default;                                   \
        Name() {}                                                              \
    }

template<typename T>
class PrimitiveWrapper
{
    T Object;
public:
    static_assert(std::is_fundamental<T>::value);
    operator T&(){return Object;}
    operator T() const {return Object;}
    PrimitiveWrapper() {}
    PrimitiveWrapper(const T& v) : Object(v) {}
    PrimitiveWrapper(T&& v) : Object(v) {}
    PrimitiveWrapper<T>& operator=(const T& v) { Object = v; return *this; }
    PrimitiveWrapper<T>& operator=(T&& v) { Object = v; return *this; }
    
    PrimitiveWrapper(const PrimitiveWrapper<T>& v) = default;
    PrimitiveWrapper(PrimitiveWrapper<T>&& v) = default;
    PrimitiveWrapper<T>& operator=(const PrimitiveWrapper<T>& v) = default;
    PrimitiveWrapper<T>& operator=(PrimitiveWrapper<T>&& v) = default;
};
}
#endif // EPI_PRiMITIVE_WRAPPER_HPP
