#pragma once
#include "restraint.hpp"
#include "rigidbody.hpp"
#include "solver.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

namespace epi {

/*
 * \brief used to process collision detection and resolution as well as
 * restraints on rigidbodies every Solver, RigidManifold and Trigger have to be
 * bound to be processed, and unbound to stop processing when destroyed all
 * objects will be automaticly unbound
 */
class PhysicsManager {
public:
    enum class eSelectMode { Min, Max, Avg };

private:
    typedef Group<
        Transform::Position, Transform::Rotation, Transform::Scale, Transform::LocalTransform, Transform::GlobalTransform,
        Rigidbody::isStaticFlag, Rigidbody::lockRotationFlag, Rigidbody::Force, Rigidbody::Velocity, Rigidbody::AngularForce, Rigidbody::AngularVelocity, Rigidbody::Mass,
        Material::AirDrag, Material::StaticFric, Material::DynamicFric, Material::Restitution,
        Collider::isTriggerFlag, Collider::ShapeModel, Collider::InertiaDevMass, Collider::Tag, Collider::Mask, Collider::ShapeTransformedPartitioned>
            CollisionManifoldGroup;
    template<class T>
    static T selectFrom(T a, T b, eSelectMode mode) {
        switch (mode) {
            case eSelectMode::Avg: return (a + b) / 2.f;
            case eSelectMode::Min: return std::min(a, b);
            case eSelectMode::Max: return std::max(a, b);
        }
    }
    typedef std::pair<Entity, Entity> ColParticipants;

    std::unique_ptr<SolverInterface> _solver =
        std::make_unique<DefaultSolver>();

    std::vector<ColParticipants> processBroadPhase(Slice<Entity, Collider::ShapeTransformedPartitioned> slice) const;
    std::vector<ColParticipants> filterBroadPhaseResults(const CollisionManifoldGroup& group, const std::vector<ColParticipants> broad_result) const;
    
    std::vector<std::vector<CollisionInfo>> detectCollisions(CollisionManifoldGroup& group, const std::vector<ColParticipants>& col_list) const;
    void solveOverlaps(CollisionManifoldGroup& group, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list) const;
    void processReactions(CollisionManifoldGroup& group, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list) const;
    //V
    void processNarrowPhase(CollisionManifoldGroup& colliding, const std::vector<ColParticipants>& col_info) const;
    
    void resetNonMovingObjects(Slice<Rigidbody::Velocity, Rigidbody::AngularVelocity, Rigidbody::Force,
         Rigidbody::AngularForce, Rigidbody::isStaticFlag, Rigidbody::lockRotationFlag> slice) const;
    void copyResultingVelocities(Slice<Entity, Rigidbody::Velocity, Rigidbody::AngularVelocity> result_slice, Rigidbody::System& rb_sys) const; 
    void copyResultingTransforms(Slice<Entity, Transform::Position, Transform::Rotation> result_slice, Transform::System& trans_sys) const; 

    //group contains only objects that can collide and react to collisions
    CollisionManifoldGroup createCollidingObjectsGroup(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                                                       Collider::System& col_sys, Material::System& mat_sys) const;
    template<class IntegratorT, class IntegrateeT>
    void integrateAny(float delT, Slice<Rigidbody::isStaticFlag, IntegratorT, IntegrateeT> slice) const {
        for(auto [isStatic, a, b] : slice) {
            if(!isStatic)
                b += a * delT;
        }
    }
    // void integrateForce          (float delT, Slice<Rigidbody::Force, Rigidbody::Velocity> slice) const;
    // void integrateAngularForce   (float delT, Slice<Rigidbody::AngularForce, Rigidbody::AngularVelocity> slice) const;
    // 
    // void integrateVelocity       (float delT, Slice<Rigidbody::Velocity, Transform::Position> slice) const;
    // void integrateAngularVelocity(float delT, Slice<Rigidbody::AngularVelocity, Transform::Rotation> slice) const;

    void applyVelocityDrag       (float delT, Slice<Rigidbody::Velocity, Material::AirDrag> slice) const;
    void applyAngularVelocityDrag(float delT, Slice<Rigidbody::AngularVelocity, Material::AirDrag> slice) const;

    void integrate(float delT, CollisionManifoldGroup& group) const;
    
    void rollbackGlobalTransform(Slice<Transform::GlobalTransform, Transform::LocalTransform> slice) const;
    void updateGlobalTransform(Slice<Transform::GlobalTransform, Transform::LocalTransform> slice) const;

public :
    // number of physics/collision steps per frame
    size_t steps = 8;

    void update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                Collider::System& col_sys, Material::System& mat_sys,
                float delT) const;

    // mode used to select bounce when colliding
    eSelectMode bounciness_select = eSelectMode::Min;
    // mode used to select friciton when colliding
    eSelectMode friction_select = eSelectMode::Min;

    template <class T>
    inline void bind() {
        _solver = std::make_unique<T>();
    }
    // size should be max simulated size
    PhysicsManager() {}
    ~PhysicsManager() {}
};
} // namespace epi
