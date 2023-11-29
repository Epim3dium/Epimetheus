#ifndef EPI_AABB_HPP
#define EPI_AABB_HPP
#include "math/math_defs.hpp"
template<class T>
struct AABB {
    sf::Vector2<T> min;
    sf::Vector2<T> max;
    sf::Vector2<T> center() const {
        return min + size() / 2;
    }
    sf::Vector2<T> size() const {
        return max - min;
    }
    T top() const {
        return max.y;
    }
    T bottom() const {
        return min.y;
    }
    T left() const {
        return min.x;
    }
    T right() const {
        return max.x;
    }
    void expandToContain(sf::Vector2<T> point) {
        min.x = std::fmin(min.x, point.x);
        min.y = std::fmin(min.y, point.y);
        max.x = std::fmax(max.x, point.x);
        max.y = std::fmax(max.y, point.y);
    }

};
typedef AABB<float> AABBf;
typedef AABB<int> AABBi;
#endif //EPI_AABB_HPP
