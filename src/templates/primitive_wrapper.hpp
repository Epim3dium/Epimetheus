#ifndef EPI_PRiMITIVE_WRAPPER_HPP
#define EPI_PRiMITIVE_WRAPPER_HPP
#include <type_traits>
namespace epi {
template<typename T>
class PrimitiveWrapper
{
    T Object;
public:
    static_assert(std::is_fundamental<T>::value);
    operator T&(){return Object;}
};
}
#endif // EPI_PRiMITIVE_WRAPPER_HPP
