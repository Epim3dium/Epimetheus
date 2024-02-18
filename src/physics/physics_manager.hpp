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
        Collider::isTriggerFlag, Collider::ShapeModel, Collider::InertiaDevMass, Collider::Tag, Collider::Mask, Collider::ShapeTransformedPartitioned> CollisionManifoldGroup;
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

    std::vector<ColParticipants> processBroadPhase(Slice<Entity, Collider::ShapeTransformedPartitioned> slice);
    void processNarrowPhase(const std::vector<ColParticipants>& col_info);
    void resetMaskedVelocities(
        Slice<Rigidbody::Velocity, Rigidbody::AngularVelocity, Rigidbody::Force, Rigidbody::AngularForce> slice,
        const std::vector<bool>& mask);

    //group contains only objects that can collide and react to collisions
    CollisionManifoldGroup createCollidingObjectsGroup(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                                                       Collider::System& col_sys, Material::System& mat_sys);

public :
    // number of physics/collision steps per frame
    size_t steps = 8;

    /*
     * updates all rigidbodies bound applying their velocities and resoving
     * collisions
     * @param pm is particle manager, if bound collisions for all active
     * particles will be resolved(colliding particles will become inactive)
     */
    void update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                Collider::System& col_sys, Material::System& mat_sys,
                float delT);

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
