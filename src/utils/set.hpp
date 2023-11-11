#ifndef EPI_SET_H
#define EPI_SET_H
#include <_types/_uint32_t.h>
#include <set>
#include <string>

namespace epi {

template <class T, class CMP = std::less<T>, class ALLOC = std::allocator<T>>
struct Set : public std::set<T, CMP, ALLOC> {
    Set operator^(const std::set<T, CMP, ALLOC>& s2);
    Set operator+(const std::set<T, CMP, ALLOC>& s2);
};
// implementation of templates
template <class T, class CMP, class ALLOC>
Set<T, CMP, ALLOC>
Set<T, CMP, ALLOC>::operator^(
    const std::set<T, CMP, ALLOC>& s2) {
    Set<T, CMP, ALLOC> s;
    std::set_intersection(this->begin(), this->end(), s2.begin(), s2.end(),
                          std::inserter(s, s.begin()));
    return s;
}
template <class T, class CMP, class ALLOC>
Set<T, CMP, ALLOC>
Set<T, CMP, ALLOC>::operator+(
    const std::set<T, CMP, ALLOC>& s2) {
    Set<T, CMP, ALLOC> s;
    std::set_union(this->begin(), this->end(), s2.begin(), s2.end(),
                   std::inserter(s, s.begin()));
    return s;
}

} // namespace epi
#endif // EPI_SET_H
