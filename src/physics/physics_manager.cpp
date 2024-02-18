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

static bool areCompatible(Collider::isTriggerFlag isTrig1, const Collider::Mask& mask1, const Collider::Tag& tag1,
                        Collider::isTriggerFlag isTrig2, const Collider::Mask& mask2, const Collider::Tag& tag2) 
{
    return (!isTrig1 || !isTrig2) &&
        (mask2.size() == 0 || tag1 ^ mask2) && 
        (mask1.size() == 0 || tag2 ^ mask1);
}
std::vector<PhysicsManager::ColParticipants> PhysicsManager::processBroadPhase(
    Slice<Entity, Collider::ShapeTransformedPartitioned> slice) 
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
// void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColParticipants>& col_list) {
//     for(auto ci = col_list.begin(); ci != col_list.end(); ci++) {
//         if(!areCompatible(ci->first, ci->second))
//             continue;
//         auto col_infos = _solver->detect(ci->first.transform, ci->first.collider, ci->second.transform, ci->second.collider);
//         for(auto col_info : col_infos) {
//             if(!col_info.detected) {
//                 continue;
//             }
//
//             //ci->first.collider->notify({*ci->first.collider, *ci->second.collider, col_info});
//             col_info.cn *= -1.f;
//             //ci->second.collider->notify({*ci->second.collider, *ci->first.collider, col_info});
//             col_info.cn *= -1.f;
//
//             if(ci->first.collider->isTrigger || ci->second.collider->isTrigger) {
//                 continue;
//             }
//             float restitution = selectFrom(ci->first.material->restitution, ci->second.material->restitution, bounciness_select);
//             float sfriction = selectFrom(ci->first.material->sfriction, ci->second.material->sfriction, friction_select);
//             float dfriction = selectFrom(ci->first.material->dfriction, ci->second.material->dfriction, friction_select);
//             _solver->solve(col_info, ci->first, ci->second, restitution, sfriction, dfriction);
//             // if(!ci->first.rigidbody->isStatic && !ci->second.rigidbody->isStatic)
//             //     Merge(ci->first.collider, ci->second.collider);
//         }
//     }
// }
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
    ((*group.template getComponent<TupleTypes>(owner).value() = tuple), ...);
}

PhysicsManager::CollisionManifoldGroup PhysicsManager::createCollidingObjectsGroup(Transform::System& trans_sys,
                                                                                   Rigidbody::System& rb_sys,
                                                                                   Collider::System& col_sys,
                                                                                   Material::System& mat_sys) 
{
    CollisionManifoldGroup colliding_objects;
    for(auto [o] : colliding_objects.sliceOwner<>()) {
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
                continue;
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
void PhysicsManager::update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
            Collider::System& col_sys, Material::System& mat_sys,
            float delT) 
{
    float deltaStep = delT / (float)steps;
    auto colliding_objects = createCollidingObjectsGroup(trans_sys, rb_sys, col_sys, mat_sys);
    
    auto col_list = processBroadPhase(colliding_objects.sliceOwner<Collider::ShapeTransformedPartitioned>());
    // // std::vector<bool> updateMask = createUpdateMask(col_sys.slice<Collider::isTriggerFlag>(), rb_sys.slice<Rigibody::isStaticFlag>());
    // resetMaskedVelocities(
    //     colliding_objects.slice<Rigidbody::Velocity, Rigidbody::AngularVelocity, Rigidbody::Force, Rigidbody::AngularForce>(),
    //     updateMask);
    // 
    // for(int i = 0; i < steps; i++) {
    //     updateRestraints(deltaStep);
    //     updateRigidbodies(deltaStep);
    //     processNarrowPhase(col_list);
    // }
    // // processSleeping();
    // for(auto r : _rigidbodies) {
    //     r.rigidbody->force = {0.f, 0.f};
    //     r.rigidbody->angular_force = 0.f;
    // }
}

} // namespace epi
