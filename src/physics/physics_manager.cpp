#include "physics_manager.hpp"
#include "collider.hpp"
#include "imgui.h"

#include "restraint.hpp"
#include "solver.hpp"
#include "rigidbody.hpp"
#include "transform.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <map>
#include <mutex>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>
#include <thread>
#include <mutex>

namespace epi {
using std::mt19937;

//from 1 to n
//veci communities(n + 1, -1);

// Collider* getHead(Collider* col) {
//     if(col->parent_collider == col)
//         return col;
//     std::vector<Collider*> to_speed;
//     while(col->parent_collider != col) {
//         to_speed.push_back(col);
//         col = col->parent_collider;
//     }
//     for(auto t : to_speed)
//         t->parent_collider = col;
//     return col;
// }
//
// void Merge(Collider* a, Collider* b) {
//     a = getHead(a);
//     b = getHead(b);
//     if(a != b) {
//         b->parent_collider = a;
//     }
// }
// bool Friends(Collider* a, Collider* b) {
//     return getHead(a) == getHead(b);
// }

static bool areCompatible(const Collider::Mask& mask1, const Collider::Tag& tag1,
                        const Collider::Mask& mask2, const Collider::Tag& tag2) 
{
    return 
        (mask2.size() == 0 || tag1 ^ mask2) && 
        (mask1.size() == 0 || tag2 ^ mask1);
}
std::vector<PhysicsManager::ColParticipants> PhysicsManager::processBroadPhase(
    Slice<Entity, Collider::ShapeTransformedPartitioned> slice) const 
{
    std::vector<PhysicsManager::ColParticipants> result;
    struct SwipeeInformation {
        float x_value;
        Entity owner;
        AABB aabb;
    };
    std::vector<SwipeeInformation> sweep_along_x;
    for(auto [owner, shape] : slice) {
        AABB aabb = AABB::Expandable();
        for(auto& convex : shape) {
            for(auto& point : convex) {
                aabb.expandToContain(point);
            }
        }
        sweep_along_x.push_back({aabb.min.x, owner, aabb});
        sweep_along_x.push_back({aabb.max.x, owner, aabb});
    }
    std::sort(sweep_along_x.begin(), sweep_along_x.end(),
        [](const SwipeeInformation& p1, const SwipeeInformation& p2) {
            return p1.x_value < p2.x_value;
        });
    struct OpenedSwipee {
        Entity owner;
        AABB aabb;
    };
    std::vector<OpenedSwipee> open;
    for(auto evaluated : sweep_along_x) {
        auto itr = std::find_if(open.begin(), open.end(), 
            [&](const OpenedSwipee& p) {
                return p.owner == evaluated.owner;
            });
        //delete if already was opened
        if(itr != open.end()) {
            std::swap(*itr, open.back());
            open.pop_back();
            continue;
        }
        
        //compare against all other opened 
        auto aabb = evaluated.aabb;
        for(auto other : open) {
            if(isOverlappingAABBAABB(other.aabb, aabb)) {
                assert(other.owner != evaluated.owner);
                result.push_back({evaluated.owner, other.owner});
            }
        }
        open.push_back({evaluated.owner, aabb});
    }
    return result;
}
std::vector<PhysicsManager::ColParticipants>
PhysicsManager::filterBroadPhaseResults(const CollisionManifoldGroup& group,
                                        const std::vector<ColParticipants> broad_result) const {
    std::vector<PhysicsManager::ColParticipants> compatible_collisions;
    compatible_collisions.reserve(broad_result.size());
    for(auto [entity1, entity2] : broad_result) {
        auto& mask1 = *group.cget<Collider::Mask>(entity1).value();
        auto& tag1 = *group .cget<Collider::Tag>(entity1) .value();
        auto& mask2 = *group.cget<Collider::Mask>(entity2).value();
        auto& tag2 = *group .cget<Collider::Tag>(entity2) .value();
        if(areCompatible(mask1, tag1, mask2, tag2)) {
            compatible_collisions.push_back({entity1, entity2});
        }
    }
    return compatible_collisions;
}
std::vector<std::vector<CollisionInfo>> PhysicsManager::detectCollisions(CollisionManifoldGroup& group, const std::vector<ColParticipants>& col_list) const {
    std::vector<std::vector<CollisionInfo>> result;
    result.reserve(col_list.size());
    for(auto [entity1, entity2] : col_list) {
        const auto& shape1 = (*group.cget<Collider::ShapeTransformedPartitioned>(entity1).value());
        const auto& shape2 = (*group.cget<Collider::ShapeTransformedPartitioned>(entity2).value());
        auto col_infos = _solver->detect(shape1, shape2);
        result.push_back({});
        for(auto info : col_infos) {
            if(info.detected)
                result.back().push_back(info);
        }
    }
    return result;
}
void PhysicsManager::solveOverlaps(CollisionManifoldGroup& group, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list) const {
    assert(col_list.size() == col_info.size());
    for(int i = 0; i < col_list.size(); i++) {
        auto [entity1, entity2] = col_list[i];
        auto isStatic1 = *group.get<Rigidbody::isStaticFlag>(entity1).value();
        auto& pos1 = *group.get<Transform::Position>(entity1).value();
        auto& rot1 = *group.get<Transform::Rotation>(entity1).value();
        auto isStatic2 = *group.get<Rigidbody::isStaticFlag>(entity2).value();
        auto& pos2 = *group.get<Transform::Position>(entity2).value();
        auto& rot2 = *group.get<Transform::Rotation>(entity2).value();
        for(const auto& info : col_info[i]) {
            _solver->solveOverlap(info, isStatic1, pos1, rot1, isStatic2, pos2, rot2);
        }
    }
}
void PhysicsManager::processReactions(CollisionManifoldGroup& group, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list) const {
    struct MaterialTuple {
        float sfric;
        float dfric;
        float bounce;
    };
    std::vector<MaterialTuple> selected_properties;
    for(int i = 0; i < col_list.size(); i++) {
        auto [entity1, entity2] = col_list[i];
        float bounce1 = *group.cget<Material::Restitution>(entity1).value();
        float sfric1 = *group.cget<Material::StaticFric>(entity1).value();
        float dfric1 = *group.cget<Material::DynamicFric>(entity1).value();
        float bounce2 = *group.cget<Material::Restitution>(entity2).value();
        float sfric2 = *group.cget<Material::StaticFric>(entity2).value();
        float dfric2 = *group.cget<Material::DynamicFric>(entity2).value();
        
        float restitution = selectFrom(bounce1, bounce2, bounciness_select);
        float sfriction = selectFrom(sfric1, sfric2, friction_select);
        float dfriction = selectFrom(dfric1, dfric2, friction_select);
        selected_properties.push_back({restitution, sfriction, dfriction});
    }
    for(int i = 0; i < col_list.size(); i++) {
        auto [entity1, entity2] = col_list[i];
        auto [sfric, dfric, bounce] = selected_properties[i];
        struct {
            float inv_inertia;
            float mass;
            vec2f* vel;
            float* ang_vel;
        }tmp[3];
        size_t ii = 1;
        for(auto e : {entity1, entity2}) {
            tmp[ii].mass = *group.cget<Rigidbody::Mass>(e).value();
            tmp[ii].inv_inertia = 1.f / (*group.cget<Collider::InertiaDevMass>(e).value() * tmp[ii].mass);
            tmp[ii].vel = group.get<Rigidbody::Velocity>(e).value();
            float& f = (*group.get<Rigidbody::AngularVelocity>(e).value());
            tmp[ii].ang_vel = &f;
            ii++;
        }
        for(const auto& info : col_info[i]) {
            vec2f rad1, rad2;
            auto cp = std::reduce(info.cps.begin(), info.cps.end()) / (float)info.cps.size();
            rad1 = cp - *group.get<Transform::Position>(entity1).value();
            rad2 = cp - *group.get<Transform::Position>(entity2).value();
            
            _solver->processReaction(info, sfric, dfric, bounce, 
                    tmp[1].inv_inertia, tmp[1].mass, rad1, *tmp[1].vel, *tmp[1].ang_vel, 
                    tmp[2].inv_inertia, tmp[2].mass, rad2, *tmp[2].vel, *tmp[2].ang_vel);
        }
    }
}
void PhysicsManager::processNarrowPhase(CollisionManifoldGroup& colliding, const std::vector<PhysicsManager::ColParticipants>& col_list) const {
    auto col_infos = detectCollisions(colliding, col_list);
    solveOverlaps(colliding, col_infos, col_list);
    processReactions(colliding, col_infos, col_list);
}
// void PhysicsManager::updateRigidObj(RigidManifold& man, float delT) {
//     if(man.collider->isTrigger)
//         return;
//     auto& rb = *man.rigidbody;
//     //processing dormants
//     // if(isSleepy && length(rb.velocity) < DORMANT_MIN_VELOCITY && abs(rb.angular_velocity) < DORMANT_MIN_ANGULAR_VELOCITY) {
//     //     man.collider->time_immobile += delT;
//     // }else {
//     //     man.collider->time_immobile = 0.f;
//     // }
//
//     if(isStatic){
//         rb.velocity = vec2f();
//         rb.angular_velocity = 0.f;
//         return;
//     }
//     if(rb.lockRotation) {
//         rb.angular_velocity = 0.f;
//         rb.angular_force = 0.f;
//     }
//
//     if(!nearlyEqual(qlen(rb.velocity), 0.f))
//         rb.velocity -= normal(rb.velocity) * std::clamp(qlen(rb.velocity) * man.material->air_drag, 0.f, length(rb.velocity)) * delT;
//     if(!nearlyEqual(rb.angular_velocity, 0.f))
//         rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * man.material->air_drag, 0.f, abs(rb.angular_velocity)) * delT;
//
//     rb.velocity += rb.force / rb.mass * delT;
//     rb.angular_velocity += rb.angular_force / man.collider->getInertia(rb.mass) * delT;
//     man.transform->setPos(man.transform->getPos() + rb.velocity * delT);
//     man.transform->setRot(man.transform->getRot() + rb.angular_velocity * delT);
// }

template<class ...GroupTypes, class ...TupleTypes>
void updateTypesFromArgs(Entity owner, Group<GroupTypes...> & group, TupleTypes ...tuple){
    ((*group.template get<TupleTypes>(owner).value() = tuple), ...);
}

PhysicsManager::CollisionManifoldGroup PhysicsManager::createCollidingObjectsGroup(Transform::System& trans_sys,
                                                                                   Rigidbody::System& rb_sys,
                                                                                   Collider::System& col_sys,
                                                                                   Material::System& mat_sys) const 
{
    CollisionManifoldGroup colliding_objects;
    for(auto [o] : rb_sys.sliceOwner<>()) {
        auto owner = o;
        auto rb_idx_maybe = rb_sys.sliceAll().getIndex(owner);
        assert(rb_idx_maybe.has_value());
        size_t rb_idx = rb_idx_maybe.value();
        auto all_tup = *(rb_sys.sliceAll().begin() + static_cast<std::ptrdiff_t>(rb_idx));
        std::apply([&colliding_objects, owner](auto&... values) 
            {
                colliding_objects.push_back(owner, values...);
            }, all_tup);
    }
    auto updateSystem = [&](auto sys) {
        for(auto [o] : colliding_objects.sliceOwner<>()) {
            auto owner = o;
            auto idx_maybe = sys.sliceAll().getIndex(owner);
            assert(idx_maybe.has_value());
            size_t idx = idx_maybe.value();
            auto all_tup = *(sys.sliceAll().begin() + static_cast<std::ptrdiff_t>(idx));
            std::apply([&](auto&... values) 
                {
                    updateTypesFromArgs(owner, colliding_objects, values...);
                }, all_tup);
        }
    };
    updateSystem(rb_sys);
    updateSystem(trans_sys);
    updateSystem(mat_sys);
    updateSystem(col_sys);
    return colliding_objects;
}

void PhysicsManager::resetNonMovingObjects(
    Slice<Rigidbody::Velocity, Rigidbody::AngularVelocity, Rigidbody::Force, Rigidbody::AngularForce,
          Rigidbody::isStaticFlag, Rigidbody::lockRotationFlag>
        slice) const 
{
    for(auto [vel, ang_vel, force, ang_force, is_static, lock_rot] : slice) {
        if(is_static) {
            vel = vec2f(0.f, 0.f);
            force = vec2f(0.f, 0.f);
            ang_vel = 0.f;
            ang_force = 0.f;
        }
        if(lock_rot) {
            ang_vel = 0.f;
            ang_force = 0.f;
        }
    }
}

void PhysicsManager::copyResultingVelocities(
    Slice<Entity, Rigidbody::Velocity, Rigidbody::AngularVelocity> result_slice,
    Rigidbody::System& rb_sys) const 
{
    for(auto [owner, vel, ang_vel] : result_slice) {
        //removed when processing collisions
        if(!rb_sys.contains(owner)) {
            continue;
        }
        (*rb_sys.get<Rigidbody::Velocity>(owner).value()) = vel;
        (*rb_sys.get<Rigidbody::AngularVelocity>(owner).value()) = ang_vel;
    }
    for(auto [force, ang_force] : rb_sys.slice<Rigidbody::Force, Rigidbody::AngularForce>()) {
        force = vec2f();
        ang_force = 0.f;
    }
}
void PhysicsManager::copyResultingTransforms(Slice<Entity, Transform::Position, Transform::Rotation> result_slice,
                                             Transform::System& trans_sys) const {
    for(auto [owner, pos, rot] : result_slice) {
        //removed when processing collisions
        if(!trans_sys.contains(owner)) {
            continue;
        }
        (*trans_sys.get<Transform::Position>(owner).value()) = pos;
        (*trans_sys.get<Transform::Rotation>(owner).value()) = rot;
    }
}
void PhysicsManager::applyVelocityDrag       (float delT, Slice<Rigidbody::Velocity, Material::AirDrag> slice) const  {
    for(auto [vel, drag] : slice) {
        if(!nearlyEqual(qlen(vel), 0.f))
            vel -= normal(vel) * std::clamp(qlen(vel) * drag, 0.f, length(vel)) * delT;
    }
}
void PhysicsManager::applyAngularVelocityDrag(float delT, Slice<Rigidbody::AngularVelocity, Material::AirDrag> slice) const {
    for(auto [ang_vel, drag] : slice) {
        if(!nearlyEqual(ang_vel, 0.f))
            ang_vel -= std::copysign(1.f, ang_vel) * std::clamp(ang_vel * ang_vel * drag, 0.f, abs(ang_vel)) * delT;
    }
}
void PhysicsManager::integrate( float delT, CollisionManifoldGroup& group) const {
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::Force, Rigidbody::Velocity>              ());
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::AngularForce, Rigidbody::AngularVelocity>());

    applyVelocityDrag       (delT, group.slice<Rigidbody::Velocity, Material::AirDrag>             ());
    applyAngularVelocityDrag(delT, group.slice<Rigidbody::AngularVelocity, Material::AirDrag>      ());
    
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::Velocity, Transform::Position>           ());
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::AngularVelocity, Transform::Rotation>    ());
    for(auto [vel] : group.slice<Rigidbody::Velocity>() ){
        vel.y += delT * 500.f;
    }
}
void PhysicsManager::rollbackGlobalTransform(Slice<Transform::GlobalTransform, Transform::LocalTransform> slice) const {
    for(auto [g, l] : slice)
        g.combine(l.getInverse());
}
void PhysicsManager::updateGlobalTransform(Slice<Transform::GlobalTransform, Transform::LocalTransform> slice) const {
    for(auto [g, l] : slice)
        g.combine(l);
}

void PhysicsManager::update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
            Collider::System& col_sys, Material::System& mat_sys,
            float delT) const 
{
    float deltaStep = delT / (float)steps;
    auto objects = createCollidingObjectsGroup(trans_sys, rb_sys, col_sys, mat_sys);
    
    resetNonMovingObjects( objects.slice<Rigidbody::Velocity, Rigidbody::AngularVelocity, Rigidbody::Force,
                                Rigidbody::AngularForce, Rigidbody::isStaticFlag, Rigidbody::lockRotationFlag>());
    
    for(int i = 0; i < steps; i++) {
        integrate(deltaStep, objects);
        
        rollbackGlobalTransform(objects.slice<Transform::GlobalTransform, Transform::LocalTransform>());
        Transform::updateLocalTransforms(objects.slice<Transform::Position, Transform::Rotation, Transform::Scale, Transform::LocalTransform>());
        updateGlobalTransform(objects.slice<Transform::GlobalTransform, Transform::LocalTransform>());
        
        Collider::updateCollisionShapes(objects.sliceOwner<Collider::ShapeModel, Collider::ShapeTransformedPartitioned>(),
                objects.slice<Transform::GlobalTransform>());
        
        auto potential_col_list = processBroadPhase(objects.sliceOwner<Collider::ShapeTransformedPartitioned>());
        auto col_list = filterBroadPhaseResults(objects, potential_col_list);
        
        processNarrowPhase(objects, col_list);
    }
    copyResultingTransforms(objects.sliceOwner<Transform::Position, Transform::Rotation>(), trans_sys);
    copyResultingVelocities(objects.sliceOwner<Rigidbody::Velocity, Rigidbody::AngularVelocity>(), rb_sys);
    // processSleeping();
}

} // namespace epi
