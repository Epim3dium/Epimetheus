#include "transform.hpp"
namespace epi {
namespace Transform {

void updateLocalTransforms(
    Slice<Position, Rotation, Scale, LocalTransform> slice) {
    for (auto [pos, rot, scale, trans] : slice) {
        trans = {sf::Transform::Identity};
        trans.translate(pos);
        trans.rotate(rot);
        trans.scale(scale);
    }
}
std::vector<vec2f> transformPoints(std::vector<vec2f> points, const sf::Transform& transform) {
    for(auto& p : points) {
        p = transform.transformPoint(p);
    }
    return points;
}
}

}
