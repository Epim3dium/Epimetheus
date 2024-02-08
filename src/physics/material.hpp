#pragma once
#include "math/types.hpp"
namespace epi {
// basic class to store physical material properties
struct Material {
    float restitution = 0.0f;
    float sfriction = 0.8f;
    float dfriction = 0.4f;
    float air_drag = 0.0001f;
};
}
