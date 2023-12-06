#ifndef EPI_GEOMETRY_FUNC_HPP
#define EPI_GEOMETRY_FUNC_HPP
#include "math_defs.hpp"
#include "aabb.hpp"
#include <vector>

namespace epi {

bool isPointInPolygon(vec2f point, const std::vector<vec2f>& polygon);
AABB<float> AABBfromPolygon(const std::vector<vec2f>& polygon);
float calcTriangleVolume(vec2f a, vec2f b, vec2f c);
vec2f findClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point);
} // namespace epi
#endif
