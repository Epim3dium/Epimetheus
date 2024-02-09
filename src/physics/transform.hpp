#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector2.hpp"
#include "math/types.hpp"

namespace epi {

typedef sf::Transform Transform;
std::vector<vec2f> transformPoints(std::vector<vec2f> points, const Transform& transform);

}
