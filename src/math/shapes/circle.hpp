#ifndef EPI_CIRCLE_HPP
#define EPI_CIRCLE_HPP
#include "math/math_defs.hpp"
namespace epi {

struct Circle {

    vec2f pos;
    float radius;
    Circle(vec2f p = vec2f(0, 0), float r = 1.f) : pos(p), radius(r) {}
};
}
#endif
