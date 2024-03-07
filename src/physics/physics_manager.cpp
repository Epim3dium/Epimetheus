#include "physics_manager.hpp"
#include "collider.hpp"
#include "imgui.h"


#include "debug/log.hpp"
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
    OwnerSlice<Collider::ShapeTransformedPartitioned> slice) const 
{
    std::vector<PhysicsManager::ColParticipants> result;
    struct SwipeeInformation {
        float x_value;
        size_t idx;
        AABB aabb;
    };
    std::vector<SwipeeInformation> sweep_along_x;
    size_t index = 0;
    for(auto [owner, shape] : slice) {
        AABB aabb = AABB::Expandable();
        for(auto& convex : shape) {
            for(auto& point : convex) {
                aabb.expandToContain(point);
            }
        }
        sweep_along_x.push_back({aabb.min.x, index, aabb});
        sweep_along_x.push_back({aabb.max.x, index, aabb});
        index++;
    }
    std::sort(sweep_along_x.begin(), sweep_along_x.end(),
        [](const SwipeeInformation& p1, const SwipeeInformation& p2) {
            return p1.x_value < p2.x_value;
        });
    struct OpenedSwipee {
        size_t idx;
        AABB aabb;
    };
    std::vector<OpenedSwipee> open;
    for(auto evaluated : sweep_along_x) {
        auto itr = std::find_if(open.begin(), open.end(), 
            [&](const OpenedSwipee& p) {
                return p.idx == evaluated.idx;
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
                assert(other.idx != evaluated.idx);
                result.push_back({evaluated.idx, other.idx});
            }
        }
        open.push_back({evaluated.idx, aabb});
    }
    return result;
}
std::vector<PhysicsManager::ColParticipants>
PhysicsManager::filterBroadPhaseResults(Slice<Rigidbody::isStaticFlag, Collider::Mask, Collider::Tag> comp_info,
                                        const std::vector<ColParticipants> broad_result) const {
    std::vector<PhysicsManager::ColParticipants> compatible_collisions;
    compatible_collisions.reserve(broad_result.size());
    for(auto [idx1, idx2] : broad_result) {
        auto [isStat1, mask1, tag1] = *(comp_info.begin() + idx1); 
        auto [isStat2, mask2, tag2] = *(comp_info.begin() + idx2); 
        if(isStat1 && isStat2)
            continue;
        if(areCompatible(mask1, tag1, mask2, tag2)) {
            compatible_collisions.push_back({idx1, idx2});
        }
    }
    return compatible_collisions;
}
std::vector<std::vector<CollisionInfo>> PhysicsManager::detectCollisions(Slice<Collider::ShapeTransformedPartitioned> shapes, const std::vector<ColParticipants>& col_list) const {
    std::vector<std::vector<CollisionInfo>> result;
    result.reserve(col_list.size());
    for(auto [idx1, idx2] : col_list) {
        auto [shape1] = *(shapes.begin() + idx1);
        auto [shape2] = *(shapes.begin() + idx2);
        auto col_infos = _solver->detect(shape1, shape2);
        result.push_back(col_infos);
    }
    return result;
}
void PhysicsManager::solveOverlaps(Slice<Rigidbody::isStaticFlag, Transform::Position, Transform::Rotation> shape_info, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list) const {
    assert(col_list.size() == col_info.size());
    for(int i = 0; i < col_list.size(); i++) {
        auto [idx1, idx2] = col_list[i];
        auto [isStatic1, pos1, rot1] = *(shape_info.begin() + idx1);
        auto [isStatic2, pos2, rot2] = *(shape_info.begin() + idx2);
        for(const auto& info : col_info[i]) {
            _solver->solveOverlap(info, isStatic1, pos1, rot1, isStatic2, pos2, rot2);
        }
    }
}
std::vector<PhysicsManager::MaterialTuple> PhysicsManager::calcSelectedMaterial(Slice<Material::Restitution, Material::StaticFric, Material::DynamicFric> mat_info, const std::vector<ColParticipants>& col_part) const {
    std::vector<MaterialTuple> selected_properties;
    for(auto [idx1, idx2] : col_part) {
        auto [bounce1, sfric1, dfric1] = *(mat_info.begin() + idx1);
        auto [bounce2, sfric2, dfric2] = *(mat_info.begin() + idx2);
        
        float restitution = selectFrom<float>(bounce1, bounce2, bounciness_select);
        float sfriction = selectFrom<float>(sfric1, sfric2, friction_select);
        float dfriction = selectFrom<float>(dfric1, dfric2, friction_select);
        selected_properties.push_back({restitution, sfriction, dfriction});
    }
    return selected_properties;
}
void PhysicsManager::processReactions(Slice < Rigidbody::isStaticFlag, Rigidbody::Mass, Rigidbody::Velocity,
                      Rigidbody::AngularVelocity, Collider::InertiaDevMass, Transform::Position> react_info,
                      const std::vector<MaterialTuple>& mat_info,
                      const std::vector<std::vector<CollisionInfo>>& col_info,
                      const std::vector<ColParticipants>& col_list) const 
{
    assert(mat_info.size() == col_info.size());
    assert(col_info.size() == col_list.size());
    for(int i = 0; i < col_list.size(); i++) {
        auto [idx1, idx2] = col_list[i];
        auto [bounce, sfric, dfric] = mat_info[i];
        auto [isStatic1, mass1, vel1, angvel1, inertia1, pos1] = *(react_info.begin() + idx1);
        auto [isStatic2, mass2, vel2, angvel2, inertia2, pos2] = *(react_info.begin() + idx2);
        float inv_inertia1 = 1.f / (inertia1 * mass1), inv_inertia2 = 1.f / (inertia2 * mass2);
        vec2f* vel_ptr1 = &vel1;
        vec2f* vel_ptr2 = &vel2;
        float* angvel_ptr1;
        float* angvel_ptr2;
        {
            float& f = angvel1;
            angvel_ptr1 = &f;
        }
        {
            float& f = angvel2;
            angvel_ptr2 = &f;
        }
        
        if(isStatic1) {
            mass1 = INFINITY;
            inv_inertia1 = 0.f;
            vel_ptr1 = nullptr;
            angvel_ptr1 = nullptr;
        }
        if(isStatic2) {
            mass2 = INFINITY;
            inv_inertia2 = 0.f;
            vel_ptr2 = nullptr;
            angvel_ptr2 = nullptr;
        }
        for(const auto& info : col_info[i]) {
            auto rad1 = info.contact_point - pos1;
            auto rad2 = info.contact_point - pos2;
            
            _solver->processReaction(info.contact_normal, sfric, dfric, bounce, 
                    inv_inertia1, mass1, rad1, vel_ptr1, angvel_ptr1, 
                    inv_inertia2, mass2, rad2, vel_ptr2, angvel_ptr2);
        }
    }
}
void PhysicsManager::processNarrowPhase(ColCompGroup& colliding, const std::vector<PhysicsManager::ColParticipants>& col_list) const {
    auto col_infos = detectCollisions(colliding.slice<Collider::ShapeTransformedPartitioned>(), col_list);
    solveOverlaps(colliding.slice<Rigidbody::isStaticFlag, Transform::Position, Transform::Rotation>(), col_infos, col_list);
    auto mat_info = calcSelectedMaterial(colliding.slice<Material::Restitution, Material::StaticFric, Material::DynamicFric>(), col_list);
    processReactions(colliding.slice< Rigidbody::isStaticFlag, Rigidbody::Mass, Rigidbody::Velocity,
                          Rigidbody::AngularVelocity, Collider::InertiaDevMass, Transform::Position>(), mat_info, col_infos, col_list);
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
    ((group.template get<TupleTypes>(owner) = tuple), ...);
}

PhysicsManager::ColCompGroup PhysicsManager::createCollidingObjectsGroup(Transform::System& trans_sys,
                                                                                   Rigidbody::System& rb_sys,
                                                                                   Collider::System& col_sys,
                                                                                   Material::System& mat_sys) const 
{
    ColCompGroup colliding_objects;
    for(auto [o] : rb_sys.sliceOwner<>()) {
        auto owner = o;
        
        auto rb_idx_maybe = rb_sys.sliceAll().getIndex(owner);
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
    // updateSystem(rb_sys);
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
    OwnerSlice<Rigidbody::Velocity, Rigidbody::AngularVelocity> result_slice,
    Rigidbody::System& rb_sys) const 
{
    for(auto [owner, vel, ang_vel] : result_slice) {
        //removed when processing collisions
        if(!rb_sys.contains(owner)) {
            continue;
        }
        (rb_sys.get<Rigidbody::Velocity>(owner)) = vel;
        (rb_sys.get<Rigidbody::AngularVelocity>(owner)) = ang_vel;
    }
    for(auto [force, ang_force] : rb_sys.slice<Rigidbody::Force, Rigidbody::AngularForce>()) {
        force = vec2f();
        ang_force = 0.f;
    }
}
void PhysicsManager::copyResultingTransforms(OwnerSlice<Transform::Position, Transform::Rotation> result_slice,
                                             Transform::System& trans_sys) const {
    for(auto [owner, pos, rot] : result_slice) {
        //removed when processing collisions
        if(!trans_sys.contains(owner)) {
            continue;
        }
        (trans_sys.get<Transform::Position>(owner)) = pos;
        (trans_sys.get<Transform::Rotation>(owner)) = rot;
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
void PhysicsManager::integrate( float delT, ColCompGroup& group) const {
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::Force, Rigidbody::Velocity>              ());
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::AngularForce, Rigidbody::AngularVelocity>());

    applyVelocityDrag       (delT, group.slice<Rigidbody::Velocity, Material::AirDrag>             ());
    applyAngularVelocityDrag(delT, group.slice<Rigidbody::AngularVelocity, Material::AirDrag>      ());
    
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::Velocity, Transform::Position>           ());
    integrateAny          (delT, group.slice  <Rigidbody::isStaticFlag, Rigidbody::AngularVelocity, Transform::Rotation>    ());
    for(auto [vel] : group.slice<Rigidbody::Velocity>() ){
        static constexpr float gravity = 1000.f;
        vel.y += delT * gravity;
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
    
    Collider::calcParitionedShapes(objects.slice<Collider::ShapeModel, Collider::ShapePartitioned>());
    Collider::updatePartitionedTransformedShapes(
        objects.sliceOwner<Collider::ShapePartitioned, Collider::ShapeTransformedPartitioned>(),
        objects.slice<Transform::GlobalTransform>());

    
    auto potential_col_list = processBroadPhase(objects.sliceOwner<Collider::ShapeTransformedPartitioned>());
    auto col_list = filterBroadPhaseResults(objects.slice<Rigidbody::isStaticFlag, Collider::Mask, Collider::Tag>(), potential_col_list);
    for (int i = 0; i < steps; i++) {
        integrate(deltaStep, objects);
        
        rollbackGlobalTransform         (objects.slice<Transform::GlobalTransform, Transform::LocalTransform>                                ());
        Transform::updateLocalTransforms(objects.slice<Transform::Position, Transform::Rotation, Transform::Scale, Transform::LocalTransform>());
        updateGlobalTransform           (objects.slice<Transform::GlobalTransform, Transform::LocalTransform>                                ());
        
        Collider::updatePartitionedTransformedShapes(
                objects.sliceOwner<Collider::ShapePartitioned, Collider::ShapeTransformedPartitioned>(),
                objects.slice<Transform::GlobalTransform>());
        
        
        processNarrowPhase(objects, col_list);
    }
    copyResultingTransforms(objects.sliceOwner<Transform::Position, Transform::Rotation>(), trans_sys);
    copyResultingVelocities(objects.sliceOwner<Rigidbody::Velocity, Rigidbody::AngularVelocity>(), rb_sys);
    // processSleeping();
}

} // namespace epi
