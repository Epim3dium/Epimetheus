#pragma once
#include "imgui.h"
#include "transform.hpp"
#include "collider.hpp"
#include "material.hpp"

#include "math/types.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <set>

namespace epi {

namespace Rigibody {
EPI_WRAP_TYPE(PrimitiveWrapper<bool>, isStaticFlag);
EPI_WRAP_TYPE(PrimitiveWrapper<bool>, lockRotationFlag);
EPI_WRAP_TYPE(vec2f, Force);
EPI_WRAP_TYPE(vec2f, Velocity);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, AngularForce);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, AngularVelocity);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, Mass);

typedef Entity Manifold;
typedef Group<isStaticFlag, lockRotationFlag, Force, Velocity, AngularForce, AngularVelocity, Mass> System;
/*
* a class holding all of tha basic rigidbody properties needed to compute collision response
*/
// class Rigidbody {
// public:
//
//     bool isStatic = false;
//     bool lockRotation = false;
//
//     vec2f force;
//     vec2f velocity;
//     float angular_force = 0.f;
//     float angular_velocity = 0.f;
//     float mass = 1.f;
//
//
//     inline void addForce(vec2f force) {
//         velocity += force / mass;
//     }
//     Rigidbody() {}
//     ~Rigidbody() {
//     }
// };
};
}
