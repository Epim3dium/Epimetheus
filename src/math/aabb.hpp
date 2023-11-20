#ifndef EPI_AABB_HPP
#define EPI_AABB_HPP
#include "math/math_defs.hpp"
struct AABB {
    vec2i min;
    vec2i max;
    vec2i center() const {
        return min + size() / 2;
    }
    vec2i size() const {
        return max - min;
    }
    float top() const {
        return max.y;
    }
    float bottom() const {
        return min.y;
    }
    float left() const {
        return min.x;
    }
    float right() const {
        return max.x;
    }
    void expandToContain(vec2i point) {
        min.x = std::fmin(min.x, point.x);
        min.y = std::fmin(min.y, point.y);
        max.x = std::fmax(max.x, point.x);
        max.y = std::fmax(max.y, point.y);
    }

};
#endif //EPI_AABB_HPP
