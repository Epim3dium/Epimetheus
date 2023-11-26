#include "math_func.hpp"
#include "math_defs.hpp"
#include <algorithm>

void epi::sort_clockwise(std::vector<sf::Vector2f>::iterator begin,
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
float epi::angle(sf::Vector2f pivot, sf::Vector2f a, sf::Vector2f b) {
    return atan2(b.y - pivot.y, b.x - pivot.x) -
           atan2(a.y - pivot.y, a.x - pivot.x);
}
sf::Vector2f epi::rotate(sf::Vector2f vec, float angle) {
    return { 
        cosf(angle) * vec.x - sinf(angle) * vec.y, 
        sinf(angle) * vec.x + cosf(angle) * vec.y, 
    };
}
float epi::length(sf::Vector2f v) {
    return sqrt(v.x * v.x + v.y * v.y);
}
float epi::dot(sf::Vector2f a, sf::Vector2f b) {
    return  a.x * b.x + a.y * b.y;
}
