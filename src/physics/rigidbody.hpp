#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
#include "collider.hpp"
#include "material.hpp"

#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <set>

namespace epi {

/*
* a class holding all of tha basic rigidbody properties needed to compute collision response
*/
class Rigidbody {
public:

    bool isStatic = false;
    bool lockRotation = false;

    vec2f force;
    vec2f velocity;
    float angular_force = 0.f;
    float angular_velocity = 0.f;
    float mass = 1.f;


    inline void addForce(vec2f force) {
        velocity += force / mass;
    }
    Rigidbody() {}
    ~Rigidbody() {
    }
};
struct RigidManifold {
private:
public:
    Transform* transform;
    Collider* collider;
    Rigidbody* rigidbody;
    Material* material;
    bool operator<(const RigidManifold& other) const {
        return transform > other.transform;
    }
    bool operator==(const RigidManifold& other) const {
        return transform == other.transform;
    }
};
}
