#pragma once
#include "col_utils.hpp"

#include "collider.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"

#include "types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace epi {

class SolverInterface {
public:
    virtual std::vector<CollisionInfo> detect(Transform* trans1, Collider* col1, Transform* trans2, Collider* col2) = 0;
    virtual void solve(CollisionInfo info, RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction) = 0;
};
class DefaultSolver : public SolverInterface {
private:
    static vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    static float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);

    static void processReaction(const CollisionInfo& info, const RigidManifold& rb1, 
           const RigidManifold& rb2,float bounce, float sfric, float dfric);
public:

    std::vector<CollisionInfo> detect(Transform* trans1, Collider* col1, Transform* trans2, Collider* col2) override;
    void solve(CollisionInfo info, RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction) override;
};

}
