#ifndef EPI_AABB_HPP
#define EPI_AABB_HPP

#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/System/Vector3.hpp"
#include "math/math_defs.hpp"

#include <cmath>
#include <math.h>
#include <numeric>
#include <vector>

namespace epi {
struct Circle;
struct ConvexPolygon;


struct AABB {
    vec2f min;
    vec2f max;

    vec2f bl() const {
        return min;
    }
    vec2f br() const {
        return vec2f(max.x, min.y);
    }
    vec2f tl() const {
        return vec2f(min.x, max.y);
    }
    vec2f tr() const {
        return max;
    }

    vec2f center() const {
        return min + (max-min) / 2.f;
    }
    float right() const {
        return max.x;
    };
    float left() const {
        return min.x;
    };
    float bottom() const {
        return min.y;
    };
    float top() const {
        return max.y;
    };
    vec2f size() const {
        return max - min;
    }
    void setCenter(vec2f c) {
        auto t = size();
        min = c - t / 2.f;
        max = c + t / 2.f;
    }
    void expandToContain(sf::Vector2f point) {
        min.x = std::fmin(min.x, point.x);
        min.y = std::fmin(min.y, point.y);
        max.x = std::fmax(max.x, point.x);
        max.y = std::fmax(max.y, point.y);
    }
    void setSize(vec2f s) {
        auto t = center();
        min = t - s / 2.f;
        max = t + s / 2.f;
    }
    AABB combine(AABB val) {
        val.min.x = std::min(min.x, val.min.x);
        val.min.y = std::min(min.y, val.min.y);
        val.max.x = std::max(max.x, val.max.x);
        val.max.y = std::max(max.y, val.max.y);
        return val;
    }
    friend void draw(sf::RenderWindow& rw, const AABB& aabb, sf::Color clr);
    friend void drawFill(sf::RenderTarget& rw, const AABB& aabb, sf::Color clr);
    friend void drawOutline(sf::RenderTarget& rw, const AABB& aabb, sf::Color clr);

    static AABB CreateMinMax(vec2f min, vec2f max);
    static AABB CreateCenterSize(vec2f center, vec2f size);
    static AABB CreateMinSize(vec2f min, vec2f size);
    static AABB CreateFromCircle(const Circle& c);
    static AABB CreateFromPolygon(const ConvexPolygon& p);
    static AABB CreateFromVerticies(const std::vector<vec2f>& verticies);
};

}
#endif
