#include "collider.hpp"
namespace epi {
namespace Collider {

std::vector<std::vector<vec2f>> transformPartitionShape(const std::vector<vec2f>& model_points, const sf::Transform& transform) {
    auto transformed_points = Transform::transformPoints(model_points, transform);
    if(!isTriangulable(transformed_points))
        std::reverse(transformed_points.begin(), transformed_points.end());

    assert(isTriangulable(transformed_points));
    
    auto triangles = triangulate(transformed_points);
    auto convex_polygons = partitionConvex(triangles);
    return convex_polygons;
}
float calcInertiaDevMass(const std::vector<std::vector<vec2f>>& model_points) {
    float result = 0.f;
    for(auto& poly : model_points) {
        result += calculateInertia(poly, 1.f);
    }
    return result;
}
void updateCollisionShapes(OwnerSlice<ShapeModel, ShapeTransformedPartitioned> shape_slice, Slice<Transform::GlobalTransform> transform_slice) {
    for(auto [owner, model, result] : shape_slice) {
        auto transform_maybe = transform_slice.get<Transform::GlobalTransform>(owner);
        assert(transform_maybe.has_value());
        result = transformPartitionShape(model, *transform_maybe.value());
    }
}

}
}
