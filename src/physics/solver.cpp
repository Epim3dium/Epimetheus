#include "solver.hpp"
#include "collider.hpp"
#include "rigidbody.hpp"
#include "debug/log.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <random>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace epi {

CollisionInfo detectOverlap(const ConvexPolygon& p1, const ConvexPolygon& p2) {
    auto intersection = intersectPolygonPolygon(p1, p2);
    if(intersection.detected) {
        return {true, intersection.contact_normal, intersection.cp, intersection.overlap};
    }
    return {false};
}
void DefaultSolver::processReaction(vec2f cn, float sfric, float dfric, float bounce, 
        float inv_inertia1, float mass1, vec2f rad1, vec2f* vel1_ptr, float* ang_vel1_ptr,
        float inv_inertia2, float mass2, vec2f rad2, vec2f* vel2_ptr, float* ang_vel2_ptr) {

    auto vel1 = (vel1_ptr == nullptr ? vec2f(0, 0) : *vel1_ptr);
    auto ang_vel1 = (ang_vel1_ptr == nullptr ? 0.f : *ang_vel1_ptr);
    auto vel2 = (vel2_ptr == nullptr ? vec2f(0, 0) : *vel2_ptr);
    auto ang_vel2 = (ang_vel2_ptr == nullptr ? 0.f : *ang_vel2_ptr);
    
    vec2f impulse (0, 0);

    vec2f rad1perp(-rad1.y, rad1.x);
    vec2f rad2perp(-rad2.y, rad2.x);

    vec2f p1ang_vel_lin = rad1perp * ang_vel1;
    vec2f p2ang_vel_lin = rad2perp * ang_vel2;

    vec2f vel_sum1 = vel1 + p1ang_vel_lin;
    vec2f vel_sum2 = vel2 + p2ang_vel_lin;

    //calculate relative velocity
    vec2f rel_vel = vel_sum2 - vel_sum1;

    float j = getReactImpulse(rad1perp, inv_inertia1, mass1, rad2perp, inv_inertia2, mass2, bounce, rel_vel, cn);
    vec2f fj = getFricImpulse(inv_inertia1, mass1, rad1perp, inv_inertia2, mass2, rad2perp, sfric, dfric, j, rel_vel, cn);
    impulse += cn * j - fj;

    if(vel1_ptr){
        *vel1_ptr -= impulse / mass1;
    }
    if(ang_vel1_ptr) {
        *ang_vel1_ptr += cross(impulse, rad1) * inv_inertia1;
    }
    
    if(vel2_ptr){
        *vel2_ptr += impulse / mass2;
    }
    if(ang_vel2_ptr){
        *ang_vel2_ptr -= cross(impulse, rad2) * inv_inertia2;
    }
}
float DefaultSolver::getReactImpulse(const vec2f& rad1perp, float p1inv_inertia, float mass1, const vec2f& rad2perp, float p2inv_inertia, float mass2, 
        float restitution, const vec2f& rel_vel, vec2f cn) {
    float contact_vel_mag = dot(rel_vel, cn);
    //equation from net
    float r1perp_dotN = dot(rad1perp, cn);
    float r2perp_dotN = dot(rad2perp, cn);

    float denom = 1.f / mass1 + 1.f / mass2 +
        (r1perp_dotN * r1perp_dotN) *  p1inv_inertia +
        (r2perp_dotN * r2perp_dotN) *  p2inv_inertia;

    float j = -(1.f + restitution) * contact_vel_mag;
    j /= denom;
    return j;
}
vec2f DefaultSolver::getFricImpulse(float p1inv_inertia, float mass1, vec2f rad1perp, float p2inv_inertia, float mass2, const vec2f& rad2perp, 
        float sfric, float dfric, float j, const vec2f& rel_vel, const vec2f& cn) {

    vec2f tangent = rel_vel - cn * dot(rel_vel, cn);

    if(length(tangent) < 0.01f)
        return vec2f(0, 0);
    tangent = normal(tangent);

    //equation from net
    float r1perp_dotT = dot(rad1perp, tangent);
    float r2perp_dotT = dot(rad2perp, tangent);

    float denom = 1.f / mass1 + 1.f / mass2 + 
        (r1perp_dotT * r1perp_dotT) *  p1inv_inertia +
        (r2perp_dotT * r2perp_dotT) *  p2inv_inertia;


    float contact_vel_mag = dot(rel_vel, tangent);
    float jt = -contact_vel_mag;
    jt /= denom;

    vec2f friction_impulse;
    if(abs(jt) <= abs(j * sfric)) {
        friction_impulse = tangent * -jt;
    } else {
        friction_impulse = tangent * -j * dfric;
    }
    return friction_impulse;
}
std::vector<CollisionInfo> DefaultSolver::detect(const Collider::ShapeTransformedPartitioned& col1, const Collider::ShapeTransformedPartitioned& col2) {
    std::vector<CollisionInfo> result;
    for(const auto& poly1 : col1) {
        for(const auto& poly2 : col2) {
            auto aabb1 = AABB::CreateFromVerticies(poly1);
            auto aabb2 = AABB::CreateFromVerticies(poly2);
            if(!isOverlappingAABBAABB(aabb1, aabb2))
                continue;
            result.push_back(detectOverlap(ConvexPolygon::CreateFromPoints(poly1), ConvexPolygon::CreateFromPoints(poly2)));
        }
    }
    return result; 
}
void DefaultSolver::solveOverlap(CollisionInfo info, 
        bool isStatic1, vec2f& pos1, float& rot1, 
        bool isStatic2, vec2f& pos2, float& rot2) 
{
    if(!info.detected)
        return;

    constexpr float response_coef = 0.9f;
    const float offset = info.overlap * response_coef;

    if(isStatic2) {
        pos1 += info.contact_normal * offset;
    } else if(isStatic1) {
        pos2 -= info.contact_normal * offset;
    } else {
        pos1 += info.contact_normal * offset * 0.5f;
        pos2 -= info.contact_normal * offset * 0.5f;
    }
}
// void DefaultSolver::solve(CollisionInfo man, RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction)  {
//     if(!man.detected) {
//         return;
//     }
//     handleOverlap(rb1, rb2, man);
//     processReaction(man, rb1, rb2, restitution, sfriction, dfriction);
// }

}
