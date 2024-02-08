#include "restraint.hpp"
#include <cmath>
namespace epi {
//inverse kinematics?
void RestraintPointTrans::update(float delT) {
    if(a.rigidbody->isStatic)
        return;
    auto& rb = *a.rigidbody;
    float inertia = a.collider->getInertia(rb.mass);

    auto r = rotateVec(model_point_a, a.transform->getRot());
    vec2f ap = a.transform->getPos() + r;
    vec2f transp = trans->getPos() + rotateVec(model_point_trans, trans->getRot());

    auto c = ap - transp;

    vec2f cn = normal(c);
    if(qlen(c) == 0.f) {
        return;
    }

    vec2f radperp(-r.y, r.x);
    vec2f ang_vel_lin = rb.lockRotation ? vec2f(0, 0) : radperp * rb.angular_velocity;

    vec2f cVel = rb.isStatic ? vec2f(0, 0) : rb.velocity;
    auto corr = -cVel - (damping_coef / delT ) * c;
    float rperp_dotN = dot(radperp, cn);
    float denom = 1.f / rb.mass +
        (rperp_dotN * rperp_dotN) / inertia;
    corr /= denom;

    auto lambda = dot(-rb.force, c) + rb.mass * dot(rb.velocity, rb.velocity);
    lambda /= dot(c, c);

    rb.force += lambda * c;
    rb.velocity += corr / rb.mass; 
    rb.angular_velocity -= cross(corr, r) / inertia;
    rb.angular_force -= cross(lambda * c, r);
}
void RestraintRigidRigid::update(float delT) {
    float inertiaA = a.collider->getInertia(a.rigidbody->mass);
    float inertiaB = b.collider->getInertia(b.rigidbody->mass);
    auto ra = rotateVec(model_point_a, a.transform->getRot());
    vec2f ap = a.transform->getPos() + ra;

    auto rb = rotateVec(model_point_b, b.transform->getRot());
    vec2f bp = b.transform->getPos() + rb;

    vec2f radperpA(-ra.y, ra.x);
    vec2f ang_vel_linA = a.rigidbody->lockRotation ? vec2f(0, 0) : radperpA * a.rigidbody->angular_velocity;
    vec2f radperpB(-rb.y, rb.x);
    vec2f ang_vel_linB = b.rigidbody->lockRotation ? vec2f(0, 0) : radperpB * b.rigidbody->angular_velocity;
    auto rel_vel = (ang_vel_linA + a.rigidbody->velocity) - (ang_vel_linB + b.rigidbody->velocity);

    auto avg_vel = (a.rigidbody->velocity + b.rigidbody->velocity) / 2.f;
    auto avg_force = (a.rigidbody->force + b.rigidbody->force) / 2.f;
    auto cA = ap - bp;
    auto cB = bp - ap;
    vec2f cn = normal(cA);
    if(qlen(cA) == 0.f) {
        return;
    }

    auto rel_velA = a.rigidbody->velocity - avg_vel;
    auto rel_velB = b.rigidbody->velocity - avg_vel;

    auto corrA = -rel_velA - (damping_coef / delT ) * cA;
    auto corrB = -rel_velB - (damping_coef / delT ) * cB;

    float rperp_dotNA = dot(radperpA, cn);
    float rperp_dotNB = dot(radperpB, -cn);
    float denom = 1.f / a.rigidbody->mass + 1.f / b.rigidbody->mass +
        (rperp_dotNB * rperp_dotNB) / inertiaB+
        (rperp_dotNA * rperp_dotNA) / inertiaA;
    corrA /= denom;
    corrB /= denom;


    auto lambdaA = dot(-a.rigidbody->force + avg_force, cA) + a.rigidbody->mass 
        * dot(rel_velA, rel_velA);
    lambdaA /= dot(cA, cA);
    auto lambdaB = dot(-b.rigidbody->force + avg_force, cB) + b.rigidbody->mass
        * dot(rel_velB, rel_velB);
    lambdaB /= dot(cB, cB);

    a.rigidbody->force += lambdaA * cA;
    a.rigidbody->velocity += corrA / a.rigidbody->mass;
    a.rigidbody->angular_force -= cross(lambdaA * cA, ra);
    a.rigidbody->angular_velocity -= cross(corrA, ra) / inertiaA;

    b.rigidbody->force += lambdaB * cB;
    b.rigidbody->velocity += corrB / b.rigidbody->mass;
    b.rigidbody->angular_force -= cross(lambdaB * cB, rb);
    b.rigidbody->angular_velocity -= cross(corrB, rb) / inertiaB;
}
}
