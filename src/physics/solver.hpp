#pragma once

#include "collider.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"

#include "math/types.hpp"
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
namespace epi {

class SolverInterface {
public:
    virtual std::vector<CollisionInfo> detect(const Collider::ShapeTransformedPartitioned& col1, const Collider::ShapeTransformedPartitioned& col2) = 0;
    virtual std::pair<vec2f, vec2f> solveOverlap(const CollisionInfo& info, bool isStatic1, vec2f pos1, bool isStatic2, vec2f pos2) = 0;
    //if any object is static, then:
    //mass = inf
    //vel = 0
    //ang_vel = 0
    //inv_inertia = 0
    //if any object has lockedRotation, then:
    //inv_inertia = 0
    //ang_vel = 0
    struct ReactionResponse {
        vec2f vel_change1;
        float angvel_change1;
        vec2f vel_change2;
        float angvel_change2;
    };
    virtual ReactionResponse processReaction(vec2f contact_normal, float sfric, float dfric, float bounce, 
            float inv_inertia1, float mass1, vec2f rad1, vec2f vel1, float ang_vel1,
            float inv_inertia2, float mass2, vec2f rad2, vec2f vel2, float ang_vel2) = 0;
    
    virtual ~SolverInterface() {}
};
class DefaultSolver : public SolverInterface {
private:
    static vec2f getFricImpulse(float p1inertia, float mass1, vec2f rad1perp, float p2inertia, float mass2, const vec2f& rad2perp, 
            float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn);
    static float getReactImpulse(const vec2f& rad1perp, float p1inertia, float mass1, const vec2f& rad2perp, float p2inertia, float mass2, 
            float restitution, const vec2f& rel_vel, vec2f cn);
public:

    std::vector<CollisionInfo> detect(const Collider::ShapeTransformedPartitioned& col1, const Collider::ShapeTransformedPartitioned& col2) override;
    std::pair<vec2f, vec2f> solveOverlap(const CollisionInfo& info, 
            bool isStatic1, vec2f pos1, 
            bool isStatic2, vec2f pos2) override;
    ReactionResponse processReaction(vec2f contact_normal, float sfric, float dfric, float bounce, 
            float inv_inertia1, float mass1, vec2f rad1, vec2f vel1, float ang_vel1,
            float inv_inertia2, float mass2, vec2f rad2, vec2f vel2, float ang_vel2) override;
};

}
