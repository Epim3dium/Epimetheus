#include "math_func.hpp"
#include "math_defs.hpp"
#include <algorithm>

namespace epi {
void sort_clockwise(std::vector<sf::Vector2f>::iterator begin,
                         std::vector<sf::Vector2f>::iterator end) {
    std::sort(begin, end, [](sf::Vector2f a, sf::Vector2f b) {
        auto anga = std::atan2(a.x, a.y);
        if (anga > EPI_PI) {
            anga -= 2.f * EPI_PI;
        } else if (anga <= -EPI_PI) {
            anga += 2.f * EPI_PI;
        }
        auto angb = std::atan2(b.x, b.y);
        if (angb > EPI_PI) {
            angb -= 2.f * EPI_PI;
        } else if (angb <= -EPI_PI) {
            angb += 2.f * EPI_PI;
        }

        return anga < angb;
    });
}
float angleAround(vec2f a, vec2f pivot, vec2f b) {
    return angle(a - pivot, b - pivot);
}
vec2f sign(vec2f x) {
    return { std::copysign(1.f, x.x), std::copysign(1.f, x.y) };
}
vec2f rotateVec(vec2f vec, float angle) {
    return vec2f(cos(angle) * vec.x - sin(angle) * vec.y,
        sin(angle) * vec.x + cos(angle) * vec.y);
}

float qlen(vec2f v) {
    return v.x * v.x + v.y * v.y;
}
float angle(vec2f a, vec2f b) {
    return atan2(cross(a,b), dot(a,b));
}
sf::Vector2f rotate(sf::Vector2f vec, float angle) {
    return { 
        cosf(angle) * vec.x - sinf(angle) * vec.y, 
        sinf(angle) * vec.x + cosf(angle) * vec.y, 
    };
}
float length(sf::Vector2f v) {
    return sqrt(v.x * v.x + v.y * v.y);
}
float dot(sf::Vector2f a, sf::Vector2f b) {
    return  a.x * b.x + a.y * b.y;
}
vec2f normal(vec2f v) {
    return v / length(v);
}
vec2f proj(vec2f a, vec2f plane_norm) {
    return (dot(a, plane_norm) / dot(plane_norm, plane_norm)) * plane_norm;
}
float cross(vec2f a, vec2f b) {
    return a.x * b.y - b.x * a.y;
}

}
