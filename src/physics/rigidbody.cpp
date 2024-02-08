#include "rigidbody.hpp"
#include "solver.hpp"
#include "types.hpp"

namespace epi {
//only dynamic objects should be placed as the r1 argument
//if r2 is static its mass in mat2 should be infinite

//vec2f rad = cp - getCollider().getPos();
//void Rigidbody::addForce(vec2f f, vec2f rad) {
//    this->force += f;
//    if(!lockRotation)
//        angular_force -= cross(f, rad); 
//}
//void Rigidbody::addVelocity(vec2f vel, vec2f rad) {
//    if(isStatic)
//        return;
//    velocity += vel;
//    if(!lockRotation)
//        angular_velocity -= cross(vel, rad); 
//}

}
