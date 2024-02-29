#pragma once
#include "core/group.hpp"
#include "math/types.hpp"
#include "templates/primitive_wrapper.hpp"

namespace epi {
namespace Transform {
EPI_WRAP_TYPE(vec2f, Position);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, Rotation);
EPI_WRAP_TYPE(vec2f, Scale);
EPI_WRAP_TYPE(sf::Transform, LocalTransform);
EPI_WRAP_TYPE(sf::Transform, GlobalTransform);

struct System : public Group<Position, Rotation, Scale, LocalTransform, GlobalTransform> {
    System() {
        setDefault<Rotation>({0.f});
        setDefault<Scale>({vec2f(1.f, 1.f)});
        setDefault<LocalTransform>({sf::Transform::Identity});
        setDefault<GlobalTransform>({sf::Transform::Identity});
    }
};
std::vector<vec2f> transformPoints(std::vector<vec2f> points, const sf::Transform& transform);
void updateLocalTransforms(
    Slice<Position, Rotation, Scale, LocalTransform> slice);
};

}
