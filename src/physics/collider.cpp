#include "collider.hpp"
namespace epi {
namespace Collider {

std::vector<std::vector<vec2f>> partitionShape(std::vector<vec2f> model_points) {
    if(!isTriangulable(model_points))
        std::reverse(model_points.begin(), model_points.end());
    
    assert(isTriangulable(model_points));
    
    auto triangles = triangulate(model_points);
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
void calcParitionedShapes(Slice<ShapeModel, ShapePartitioned> shape_slice) {
    for(auto [model, partitined] : shape_slice) {
        // auto transform_maybe = transform_slice.get<Transform::GlobalTransform>(owner);
        // assert(transform_maybe.has_value());
        partitined = partitionShape(model);
    }
}
void updatePartitionedTransformedShapes(OwnerSlice<ShapePartitioned, ShapeTransformedPartitioned> shape_slice, Slice<Transform::GlobalTransform> transform_slice){
    for(auto [owner, partitioned, trans_part] : shape_slice) {
        assert(transform_slice.contains(owner)) ;
        const auto& trans = transform_slice.get<Transform::GlobalTransform>(owner);
        
        trans_part = partitioned;
        for(auto& convex : trans_part) {
            convex = Transform::transformPoints(convex, trans);
        }
    }
}

}
}
