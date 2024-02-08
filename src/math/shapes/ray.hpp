#ifndef EPI_RAY_HPP
#define EPI_RAY_HPP
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
namespace epi {
struct Ray {
    vec2f pos;
    vec2f dir;

    float length() const {
        return epi::length(dir);
    }
    Ray() {}
    static Ray CreatePoints(vec2f a, vec2f b);
    static Ray CreatePositionDirection(vec2f p, vec2f d);
};

}
#endif
