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

static bool areCompatible(const Collider::Mask& mask1, const Collider::Tag& tag1,
                        const Collider::Mask& mask2, const Collider::Tag& tag2) 
{
    return 
        (mask2.size() == 0 || tag1 ^ mask2) && 
        (mask1.size() == 0 || tag2 ^ mask1);
}
std::vector<PhysicsManager::ColParticipants> PhysicsManager::processBroadPhase(
    OwnerSlice<ShapeTransformedPartitioned> slice) const 
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
PhysicsManager::filterBroadPhaseResults(Slice<isStaticFlag, Mask, Tag> comp_info,
                                        const std::vector<ColParticipants> broad_result) const {
    std::vector<PhysicsManager::ColParticipants> compatible_collisions;
    compatible_collisions.reserve(broad_result.size());
    for(auto [idx1, idx2] : broad_result) {
        auto [isStat1, mask1, tag1] = *(comp_info.begin() + idx1); 
        auto [isStat2, mask2, tag2] = *(comp_info.begin() + idx2); 
        if(isStat1 && isStat2)
            continue;
        if(areCompatible(mask1, tag1, mask2, tag2)) {
            compatible_collisions.push_back({std::min(idx1, idx2), std::max(idx1, idx2)});
        }
    }
    return compatible_collisions;
}
vec2f helper_getAxis(const std::vector<vec2f>& points1, const std::vector<vec2f>& points2, size_t index, bool flip) {
    auto& p = flip ? points2 : points1;
    assert(index < p.size());
    auto prev = p[index];
    auto vert = p[(index + 1) % p.size()];
    vec2f perp = vert - prev;
    auto cn = normal({ -perp.y, perp.x });
    return normal({ -perp.y, perp.x });  
}
std::vector<PhysicsManager::SeparatingAxisList>
PhysicsManager::calcSeparatingAxis(Slice<ShapeTransformedPartitioned> shapes,
                                   const std::vector<ColParticipants>& col_list, ThreadPool* tp) const {
    std::vector<PhysicsManager::SeparatingAxisList> result(col_list.size());
    auto processRange = [&](int begin, int end) {
        for(int i = begin; i < end; i++) {
            auto [idx1, idx2] = col_list[i];
            auto [shape1] = *(shapes.begin() + idx1); 
            auto [shape2] = *(shapes.begin() + idx2); 
            for(const auto& poly1 : shape1) {
                for(const auto& poly2 : shape2) {
                    auto a = calcSeparatingAxisPolygonPolygon(poly1, poly2);
                    result[i].push_back(a);
                }
            }
        }
    };
    if(tp) {
        tp->dispatch(col_list.size(), processRange);
        tp->waitForCompletion();
    }else {
        processRange(0, col_list.size());
    }
    return result;
}
std::vector<std::vector<CollisionInfo>> PhysicsManager::detectCollisions(Slice<ShapeTransformedPartitioned> shapes, const std::vector<SeparatingAxisList>& axes_list, const std::vector<ColParticipants>& col_list, ThreadPool* tp) const {
    std::vector<std::vector<CollisionInfo>> result(col_list.size());
    auto processRange = [&](int begin, int end) {
        for(size_t i = begin; i < end; i++) {
            auto [idx1, idx2] = col_list[i];
            auto [shape1] = *(shapes.begin() + idx1);
            auto [shape2] = *(shapes.begin() + idx2);
            const auto& axises = axes_list[i];
            size_t index = 0;
            result[i].resize(shape1.size() * shape2.size());
            for(const auto& poly1 : shape1) {
                for(const auto& poly2 : shape2) {
                    const auto& axis = axises[index];
                    if(axis.exists) {
                        auto projAxis = helper_getAxis(poly1, poly2, axis.edge_index, axis.wasItSecondShape);
                        auto tmp = intersectPolygonPolygonUsingAxis(poly1, poly2, projAxis, axis.wasItSecondShape); 
                        result[i][index] = {tmp.detected, tmp.contact_normal, tmp.cp, tmp.overlap};
                    }else {
                        result[i][index] = {false};
                    }
                    index++;
                }
            }
        }
    };
    if(tp) {
        tp->dispatch(col_list.size(), processRange);
        tp->waitForCompletion();
    }else {
        processRange(0, col_list.size());
    }
    return result;
}
void PhysicsManager::solveOverlaps(Slice<isStaticFlag, Position> shape_info,
                                   const std::vector<std::vector<CollisionInfo>>& col_info,
                                   const std::vector<ColParticipants>& col_list, std::vector<float>& pressure_list, ThreadPool* tp) const {
    assert(col_list.size() == col_info.size());
    auto processRange = [&](int begin, int end) {
        for (int i = begin; i < end; i++) {
            auto [idx1, idx2] = col_list[i];

            float pressure1 = pressure_list[idx1];
            float pressure2 = pressure_list[idx2];
            float denom = 2.f / (pressure1 + pressure2);

            auto [isStatic1, pos1] = *(shape_info.begin() + idx1);
            auto [isStatic2, pos2] = *(shape_info.begin() + idx2);
            float mult1 = isStatic2 ? 1.f : denom * pressure2;
            float mult2 = isStatic1 ? 1.f : denom * pressure1;
            for (const auto& info : col_info[i]) {
                auto [off1, off2] = m_solver->solveOverlap(info, isStatic1, pos1, isStatic2, pos2);
                pos1 += off1;
                pos2 += off2;
            }
        }
    };
    if(tp) {
        tp->dispatch(col_list.size(), processRange);
        tp->waitForCompletion();
    }else {
        processRange(0, col_list.size());
    }
}
std::vector<PhysicsManager::MaterialTuple>
PhysicsManager::calcSelectedMaterial(Slice<Restitution, StaticFric, DynamicFric> mat_info,
                                     const std::vector<ColParticipants>& col_part) const {
    std::vector<MaterialTuple> selected_properties;
    for (auto [idx1, idx2] : col_part) {
        auto [bounce1, sfric1, dfric1] = *(mat_info.begin() + idx1);
        auto [bounce2, sfric2, dfric2] = *(mat_info.begin() + idx2);

        float restitution = selectFrom<float>(bounce1, bounce2, bounciness_select);
        float sfriction = selectFrom<float>(sfric1, sfric2, friction_select);
        float dfriction = selectFrom<float>(dfric1, dfric2, friction_select);
        selected_properties.push_back({restitution, sfriction, dfriction});
    }
    return selected_properties;
}
#define VERY_SMALL_NUMBER (0.01)
std::vector<SolverInterface::ReactionResponse> PhysicsManager::processReactions(float delT, 
                    Slice < isStaticFlag, lockRotationFlag, Mass, Velocity,
                      AngularVelocity, InertiaDevMass, Position> react_info,
                      const std::vector<MaterialTuple>& mat_info,
                      const std::vector<std::vector<CollisionInfo>>& col_info,
                      const std::vector<ColParticipants>& col_list, ThreadPool* tp) const 
{
    std::vector<SolverInterface::ReactionResponse> result(col_list.size());
    static int ticks_passed = 0;
    ticks_passed++;
    
    assert(mat_info.size() == col_info.size());
    assert(col_info.size() == col_list.size());
    auto processRange = [&](int begin, int end) {
        for(int i = 0; i < col_list.size(); i++) {
            auto [idx1, idx2] = col_list[i];
            
            
            auto [bounce, sfric, dfric] = mat_info[i];
            auto [isStatic1, lockRot1, mass1, vel1, angvel1, inertia1, pos1] = *(react_info.begin() + idx1);
            auto [isStatic2, lockRot2, mass2, vel2, angvel2, inertia2, pos2] = *(react_info.begin() + idx2);
            
            float inv_inertia1 = 1.f / (inertia1 * mass1);
            float inv_inertia2 = 1.f / (inertia2 * mass2);
            
            if(isStatic1) {
                mass1 = INFINITY;
                inv_inertia1 = 0.f;
                vel1 = vec2f(0, 0);
                angvel1 = 0.f;
            }
            if(lockRot1) {
                inv_inertia1 = 0.f;
            }
            if(isStatic2) {
                mass2 = INFINITY;
                inv_inertia2 = 0.f;
                vel2 = vec2f(0, 0);
                angvel2 = 0.f;
            }
            if(lockRot2) {
                inv_inertia2 = 0.f;
            }
            
            const float Slop = 10 * VERY_SMALL_NUMBER / static_cast<float>(this->steps);
            for(const auto& info : col_info[i]) {
                if(info.overlap < Slop || !info.detected) {
                    continue;
                }
                ReactionResponse response;
                for(auto contact_point : info.contact_points) {
                    auto rad1 = contact_point - pos1;
                    auto rad2 = contact_point - pos2;
                    
                    auto reaction = m_solver->processReaction(info.contact_normal, sfric, dfric, bounce, 
                            inv_inertia1, mass1, rad1, vel1, angvel1, 
                            inv_inertia2, mass2, rad2, vel2, angvel2);
                    
                    if(!isStatic1) {
                        response.vel_change1 += reaction.vel_change1;
                        if(!lockRot1) {
                            response.angvel_change1 += reaction.angvel_change1;
                        }
                    }
                    if(!isStatic2) {
                        response.vel_change2 += reaction.vel_change2;
                        if(!lockRot2) {
                            response.angvel_change2 += reaction.angvel_change2;
                        }
                    }
                }
                result[i] = response;
            }
        }
    };
    if(tp) {
        tp->dispatch(col_list.size(), processRange);
        tp->waitForCompletion();
    }else {
        processRange(0, col_list.size());
    }
    return result;
}
void PhysicsManager::applyReactionReseponses(const std::vector<ReactionResponse>& responses, Slice<Velocity, AngularVelocity> velocities, std::vector<ColParticipants> col_list) const {
    for(int i = 0; i < responses.size(); i++) {
        const auto& r = responses[i];
        const auto& [idx1, idx2] = col_list[i];
        auto [vel1, angvel1] = *(velocities.begin() + idx1);
        auto [vel2, angvel2] = *(velocities.begin() + idx2);
        
        vel1 += r.vel_change1;
        angvel1 += r.angvel_change1;
        
        vel2 += r.vel_change2;
        angvel2 += r.angvel_change2;
    }
}
void PhysicsManager::processNarrowPhase(float delT, ColCompGroup& colliding, const std::vector<ColParticipants>& col_list, const std::vector<SeparatingAxisList>& axes, ThreadPool* tp) const {
    auto col_infos = detectCollisions(colliding.slice<ShapeTransformedPartitioned>(), axes, col_list, tp);
    std::vector<float> pressure_list(colliding.size(), VERY_SMALL_NUMBER);
    for(int i = 0; i < col_infos.size(); i++) {
        for(auto info : col_infos[i]) {
            if(!info.detected) {
                continue;
            }
            auto [idx1, idx2] = col_list[i];
            pressure_list[idx1] += info.overlap;
            pressure_list[idx2] += info.overlap;
        }
    }
    solveOverlaps(colliding.slice<isStaticFlag, Position>(), col_infos, col_list, pressure_list, tp);
    
    auto mat_info = calcSelectedMaterial(colliding.slice<Restitution, StaticFric, DynamicFric>(), col_list);
    auto reactions = processReactions(delT, colliding.slice< isStaticFlag, lockRotationFlag, Mass, Velocity,
                          AngularVelocity, InertiaDevMass, Position>(), mat_info, col_infos, col_list, tp);
    applyReactionReseponses(reactions, colliding.slice<Velocity, AngularVelocity>(), col_list);
}

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


void PhysicsManager::copyResultingVelocities(
    OwnerSlice<Velocity, AngularVelocity> result_slice,
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
void PhysicsManager::copyResultingTransforms(OwnerSlice<Position, Rotation> result_slice,
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
void PhysicsManager::applyVelocityDrag       (float delT, Slice<Velocity, AirDrag> slice) const  {
    for(auto [vel, drag] : slice) {
        if(!nearlyEqual(qlen(vel), 0.f))
            vel -= normal(vel) * std::clamp(qlen(vel) * drag, 0.f, length(vel)) * delT;
    }
}
void PhysicsManager::applyAngularVelocityDrag(float delT, Slice<AngularVelocity, AirDrag> slice) const {
    for(auto [ang_vel, drag] : slice) {
        if(!nearlyEqual(ang_vel, 0.f))
            ang_vel -= std::copysign(1.f, ang_vel) * std::clamp(ang_vel * ang_vel * drag, 0.f, abs(ang_vel)) * delT;
    }
}
void PhysicsManager::integrate( float delT, ColCompGroup& group) const {
    integrateAny            (delT, group.slice  <isStaticFlag, Force, Velocity>              ());
    integrateAny            (delT, group.slice  <isStaticFlag, AngularForce, AngularVelocity>());

    applyVelocityDrag       (delT, group.slice<Velocity, AirDrag>                            ());
    applyAngularVelocityDrag(delT, group.slice<AngularVelocity, AirDrag>                     ());
    
    integrateAny            (delT, group.slice  <isStaticFlag, Velocity, Position>           ());
    integrateAny            (delT, group.slice  <isStaticFlag, AngularVelocity, Rotation>    ());
    
}
void PhysicsManager::rollbackGlobalTransform(Slice<GlobalTransform, LocalTransform> slice) const {
    for(auto [g, l] : slice)
        g.combine(l.getInverse());
}
void PhysicsManager::updateGlobalTransform(Slice<GlobalTransform, LocalTransform> slice) const {
    for(auto [g, l] : slice)
        g.combine(l);
}

void PhysicsManager::update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
            Collider::System& col_sys, Material::System& mat_sys,
            float delT, ThreadPool* thread_pool) const 
{
    float deltaStep = delT / (float)steps;
    auto objects = createCollidingObjectsGroup(trans_sys, rb_sys, col_sys, mat_sys);
    
    
    Collider::calcParitionedShapes(objects.slice<ShapeModel, ShapePartitioned>());
    Collider::updatePartitionedTransformedShapes(
        objects.sliceOwner<ShapePartitioned, ShapeTransformedPartitioned>(),
        objects.slice<GlobalTransform>());

    
    auto potential_col_list = processBroadPhase(objects.sliceOwner<ShapeTransformedPartitioned>());
    auto col_list = filterBroadPhaseResults(objects.slice<isStaticFlag, Mask, Tag>(), potential_col_list);
    auto col_axis_list = calcSeparatingAxis(objects.slice<ShapeTransformedPartitioned>(), col_list, thread_pool);
    
    for (int i = 0; i < steps; i++) {
        if(i % 3 == 0) {
            col_axis_list = calcSeparatingAxis(objects.slice<ShapeTransformedPartitioned>(), col_list, thread_pool);
        }
        for(auto [vel] : objects.slice<Rigidbody::Velocity>() ){
            vel.y += gravity * deltaStep /* * mass */;
        }
        integrate(deltaStep, objects);
        rollbackGlobalTransform         (objects.slice<GlobalTransform, LocalTransform>                                ());
        Transform::updateLocalTransforms(objects.slice<Position, Rotation, Scale, LocalTransform>());
        updateGlobalTransform           (objects.slice<GlobalTransform, LocalTransform>                                ());
        
        Collider::updatePartitionedTransformedShapes(
                objects.sliceOwner<ShapePartitioned, ShapeTransformedPartitioned>(),
                objects.slice<GlobalTransform>());
        processNarrowPhase(deltaStep, objects, col_list, col_axis_list, thread_pool);
    }
    copyResultingTransforms(objects.sliceOwner<Position, Rotation>(), trans_sys);
    copyResultingVelocities(objects.sliceOwner<Velocity, AngularVelocity>(), rb_sys);
    // processSleeping();
}

} // namespace epi
