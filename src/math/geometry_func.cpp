#include "geometry_func.hpp"
namespace epi {

bool isPointInPolygon(vec2f point, const std::vector<vec2f>& polygon) {
    int i, j, nvert = polygon.size();
    bool c = false;

    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((polygon[i].y >= point.y) != (polygon[j].y >= point.y)) &&
            (point.x <= (polygon[j].x - polygon[i].x) *
                                (point.y - polygon[i].y) /
                                (polygon[j].y - polygon[i].y) +
                            polygon[i].x))
            c = !c;
    }

    return c;
}
AABB<float> AABBfromPolygon(const std::vector<vec2f>& polygon) {
    AABB<float> result;
    result.min = {INFINITY, INFINITY};
    result.max = {-INFINITY, -INFINITY};
    for(auto& p : polygon) {
        result.min.x = std::min<float>(result.min.x, p.x);
        result.max.x = std::max<float>(result.max.x, p.x);
        result.min.y = std::min<float>(result.min.y, p.y);
        result.max.y = std::max<float>(result.max.y, p.y);
    }
    return result;
}
} // namespace epi
