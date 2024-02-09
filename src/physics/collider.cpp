#include "collider.hpp"
namespace epi {

namespace Collider {
std::vector<std::vector<vec2f>> transformPartitionShape(const std::vector<vec2f>& model_points, const sf::Transform& transform) {
    auto transformed_points = Transform::transformPoints(model_points, transform);
    auto triangles = triangulate(transformed_points);
    auto convex_polygons = partitionConvex(triangles);
    return convex_polygons;
}
AABB calcAABB(const std::vector<vec2f>& model_points, const sf::Transform& transform) {
    assert(model_points.size() > 0);
    auto transformed_points = Transform::transformPoints(model_points, transform);
    AABB result = AABB::Expandable();
    for(auto p : transformed_points) {
        result.min.x = std::min(result.min.x, p.x);
        result.min.y = std::min(result.min.y, p.y);
        
        result.max.x = std::max(result.max.x, p.x);
        result.max.y = std::max(result.max.y, p.y);
    }
    return result;
}
float calcInertiaDevMass(const std::vector<std::vector<vec2f>>& model_points) {
    float result = 0.f;
    for(auto& poly : model_points) {
        result += calculateInertia(poly, 1.f);
    }
    return result;
}

}
}
