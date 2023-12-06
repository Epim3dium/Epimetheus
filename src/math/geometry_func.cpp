#include "geometry_func.hpp"
#include "math/math_func.hpp"
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
float calcTriangleVolume(vec2f a, vec2f b, vec2f c) {
    auto la = length(a - b);
    auto lb = length(b - c);
    auto lc = length(c - a);
    auto S = la + lb + lc;
    S *= 0.5f;
    return sqrt(S * (S - la) * (S - lb) * (S -lc));
}
vec2f findClosestPointOnRay(vec2f ray_origin, vec2f ray_dir, vec2f point) {
    float ray_dir_len = length(ray_dir);
    vec2f seg_v_unit = ray_dir / ray_dir_len;
    float proj = dot(point - ray_origin, seg_v_unit);
    if (proj <= 0)
        return ray_origin;
    if (proj >= ray_dir_len)
        return ray_origin + ray_dir;
    return seg_v_unit * proj + ray_origin;
}
} // namespace epi
