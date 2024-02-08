#pragma once
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>
#include <set>

#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "math/math_defs.hpp"

#include "SFML/Graphics.hpp"
#include "math/math_func.hpp"
#include "shapes/AABB.hpp"
#include "shapes/ray.hpp"
#include "shapes/circle.hpp"
#include "shapes/convex_polygon.hpp"
#include "shapes/concave_polygon.hpp"

namespace epi {
typedef sf::Color Color;

namespace PastelColor {
    const Color bg =     Color(0xa89984ff);
    const Color bg1 =    Color(0xa89984ff);
    const Color bg2 =    Color(0xbdae93ff);
    const Color bg3 =    Color(0xebdbb2ff);
    const Color bg4 =    Color(0x1e3a4cff);
    const Color Red =    Color(0x9d0006ff);
    const Color Green =  Color(0x79740eff);
    const Color Yellow = Color(0xb57614ff);
    const Color Blue =   Color(0x076678ff);
    const Color Purple = Color(0x8f3f71ff);
    const Color Aqua =   Color(0x427b58ff);
    const Color Orange = Color(0xaf3a03ff);
    const Color Gray =   Color(0x928374ff);
};

struct Circle;
class ConvexPolygon;

std::vector<ConvexPolygon> triangulate(const std::vector<vec2f>& points);
std::vector<ConvexPolygon> partitionConvex(const std::vector<ConvexPolygon>& polygons);


vec2f operator* (vec2f a, vec2f b);
}
