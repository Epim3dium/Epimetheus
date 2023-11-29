#ifndef EPI_GEOMETRY_FUNC_HPP
#define EPI_GEOMETRY_FUNC_HPP
#include "math_defs.hpp"
#include "aabb.hpp"
#include <vector>

namespace epi {

bool isPointInPolygon(vec2f point, const std::vector<vec2f>& polygon);
AABB<float> AABBfromPolygon(const std::vector<vec2f>& polygon);
} // namespace epi
#endif
