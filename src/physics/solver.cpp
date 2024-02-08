#include "solver.hpp"
#include "col_utils.hpp"
#include "collider.hpp"
#include "rigidbody.hpp"

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
        std::vector<vec2f> cps;
        cps = findContactPoints(p1, p2);
        if(cps.size() ==0)
            return {false};
        return {true, intersection.contact_normal, cps , intersection.overlap};
    }
    return {false};
}
void handleOverlap(RigidManifold& m1, RigidManifold& m2, const CollisionInfo& man) {
    if(!man.detected)
        return;
    auto& t1 = *m1.transform;
    auto& t2 = *m2.transform;

    constexpr float response_coef = 0.9f;
    constexpr float pos_rot_response_ratio = 0.8f;
    const float offset = man.overlap * response_coef;

    auto rotateByOffest = [&man](vec2f contact_point, Transform& trans, float overlap) {
        auto rad = trans.getPos() - contact_point;
        auto rad_normal = normal(rad);
        auto rot_diff = overlap / (2.f * EPI_PI * length(rad));
        trans.setRot(trans.getRot() + rot_diff * cross(rad_normal, man.cn));
    };

    float inv_cps_size = 1.f / static_cast<float>(man.cps.size());

    if(m2.rigidbody->isStatic) {
        t1.setPos(t1.getPos() + man.cn * offset * pos_rot_response_ratio);

        //for when it's 'lodged between 2 static objects'
        for(auto cp : man.cps)
            rotateByOffest(cp, t1, -offset * (1.f - pos_rot_response_ratio) * inv_cps_size);
    } else if(m1.rigidbody->isStatic) {
        t2.setPos(t2.getPos() - man.cn * offset * pos_rot_response_ratio);

        for(auto cp : man.cps)
            rotateByOffest(cp, t2, offset * (1.f - pos_rot_response_ratio) * inv_cps_size);
    } else {
        t1.setPos(t1.getPos() + man.cn * offset * 0.5f);
        t2.setPos(t2.getPos() - man.cn * offset * 0.5f);
    }
}
void DefaultSolver::processReaction(const CollisionInfo& info, const RigidManifold& m1, 
       const RigidManifold& m2,float bounce, float sfric, float dfric)
{
    //auto cp = std::reduce(info.cps.begin(), info.cps.end()) / (float)info.cps.size();

    auto& rb1 = *m1.rigidbody;
    auto& rb2 = *m2.rigidbody;

    float mass1 = rb1.isStatic ? INFINITY : rb1.mass;
    float mass2 = rb2.isStatic ? INFINITY : rb2.mass;
    float inv_inertia1 = 1.f / ((rb1.isStatic || rb1.lockRotation) ? INFINITY : m1.collider->getInertia(rb1.mass));
    float inv_inertia2 = 1.f / ((rb2.isStatic || rb2.lockRotation) ? INFINITY : m2.collider->getInertia(rb2.mass));
    vec2f vel1 = rb1.velocity;
    vec2f vel2 = rb2.velocity;
    float ang_vel1 = rb1.angular_velocity;
    float ang_vel2 = rb2.angular_velocity;


    auto cp = std::reduce(info.cps.begin(), info.cps.end()) / (float)info.cps.size();
    vec2f impulse (0, 0);

    vec2f rad1 = cp - m1.transform->getPos();
    vec2f rad2 = cp - m2.transform->getPos();

    vec2f rad1perp(-rad1.y, rad1.x);
    vec2f rad2perp(-rad2.y, rad2.x);

    vec2f p1ang_vel_lin = rb1.lockRotation ? vec2f(0, 0) : rad1perp * ang_vel1;
    vec2f p2ang_vel_lin = rb2.lockRotation ? vec2f(0, 0) : rad2perp * ang_vel2;

    vec2f vel_sum1 = rb1.isStatic ? vec2f(0, 0) : vel1 + p1ang_vel_lin;
    vec2f vel_sum2 = rb2.isStatic ? vec2f(0, 0) : vel2 + p2ang_vel_lin;

    //calculate relative velocity
    vec2f rel_vel = vel_sum2 - vel_sum1;

    float j = getReactImpulse(rad1perp, inv_inertia1, mass1, rad2perp, inv_inertia2, mass2, bounce, rel_vel, info.cn);
    vec2f fj = getFricImpulse(inv_inertia1, mass1, rad1perp, inv_inertia2, mass2, rad2perp, sfric, dfric, j, rel_vel, info.cn);
    impulse += info.cn * j - fj;

    float cps_ctr = (float)info.cps.size();
    rb1.velocity -= impulse / rb1.mass;
    rb1.angular_velocity += cross(impulse, rad1) * inv_inertia1;
    rb2.velocity += impulse / rb2.mass;
    rb2.angular_velocity -= cross(impulse, rad2) * inv_inertia2;
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
    else
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
std::vector<CollisionInfo> DefaultSolver::detect(Transform* trans1, Collider* col1, Transform* trans2, Collider* col2) {
    std::vector<CollisionInfo> result;
    for(const auto& poly1 : col1->getPolygonShape(*trans1)) {
        for(const auto& poly2 : col2->getPolygonShape(*trans2)) {
            auto aabb1 = AABB::CreateFromVerticies(poly1.getVertecies());
            auto aabb2 = AABB::CreateFromVerticies(poly2.getVertecies());
            if(!isOverlappingAABBAABB(aabb1, aabb2))
                continue;
            result.push_back(detectOverlap(poly1, poly2));
        }
    }
    return result; 
}
void DefaultSolver::solve(CollisionInfo man, RigidManifold rb1, RigidManifold rb2, float restitution, float sfriction, float dfriction)  {
    if(!man.detected) {
        return;
    }
    handleOverlap(rb1, rb2, man);
    processReaction(man, rb1, rb2, restitution, sfriction, dfriction);
}

}
