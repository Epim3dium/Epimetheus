#pragma once
#include "restraint.hpp"
#include "rigidbody.hpp"
#include "solver.hpp"
#include "multithreading/thread_pool.hpp"

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
    typedef Collider::ShapeTransformedPartitioned ShapeTransformedPartitioned;
    typedef Transform::GlobalTransform            GlobalTransform;
    typedef Rigidbody::Mass                       Mass;
    typedef Material::Restitution                 Restitution;
    typedef Collider::ShapePartitioned            ShapePartitioned;
    typedef Collider::Mask                        Mask;
    typedef Collider::Tag                         Tag;
    typedef Collider::InertiaDevMass              InertiaDevMass;
    typedef Collider::ShapeModel                  ShapeModel;
    typedef Collider::isTriggerFlag               isTriggerFlag;
    typedef Material::DynamicFric                 DynamicFric;
    typedef Material::StaticFric                  StaticFric;
    typedef Material::AirDrag                     AirDrag;
    typedef Rigidbody::AngularVelocity            AngularVelocity;
    typedef Rigidbody::AngularForce               AngularForce;
    typedef Rigidbody::Velocity                   Velocity;
    typedef Rigidbody::Force                      Force;
    typedef Rigidbody::lockRotationFlag           lockRotationFlag;
    typedef Rigidbody::isStaticFlag               isStaticFlag;
    typedef Transform::LocalTransform             LocalTransform;
    typedef Transform::Scale                      Scale;
    typedef Transform::Rotation                   Rotation;
    typedef Transform::Position                   Position;
    typedef Group<
        Position, Rotation, Scale, LocalTransform, GlobalTransform,
        isStaticFlag, lockRotationFlag, Force, Velocity, AngularForce, AngularVelocity, Mass,
        AirDrag, StaticFric, DynamicFric, Restitution,
        isTriggerFlag, ShapeModel, InertiaDevMass, Tag, Mask, ShapePartitioned, ShapeTransformedPartitioned>
            ColCompGroup;
    template<class T>
    static T selectFrom(T a, T b, eSelectMode mode) {
        switch (mode) {
            case eSelectMode::Avg: return (a + b) / 2.f;
            case eSelectMode::Min: return std::min(a, b);
            case eSelectMode::Max: return std::max(a, b);
        }
    }
    typedef std::pair<size_t, size_t> ColParticipants;
    typedef std::vector<SeparatingAxisInfo> SeparatingAxisList;

    std::shared_ptr<SolverInterface> m_solver =
        std::make_shared<DefaultSolver>();

    std::vector<ColParticipants> processBroadPhase(OwnerSlice<ShapeTransformedPartitioned> slice) const;
    std::vector<ColParticipants> filterBroadPhaseResults(Slice<isStaticFlag, Mask, Tag> comp_info, const std::vector<ColParticipants> broad_result) const;
    std::vector<SeparatingAxisList> calcSeparatingAxis(Slice<ShapeTransformedPartitioned> shapes, const std::vector<ColParticipants>& col_list, ThreadPool* tp) const;
    
    struct MaterialTuple {
        float bounce;
        float sfric;
        float dfric;
    };
    std::vector<std::vector<CollisionInfo>> detectCollisions(Slice<ShapeTransformedPartitioned> shapes, const std::vector<SeparatingAxisList>& axes, const std::vector<ColParticipants>& col_list, ThreadPool* tp) const;
    
    void solveOverlaps(Slice<isStaticFlag, Position> shape_info, const std::vector<std::vector<CollisionInfo>>& col_info, const std::vector<ColParticipants>& col_list, std::vector<float>& pressures) const;
    std::vector<MaterialTuple> calcSelectedMaterial(Slice<Restitution, StaticFric, DynamicFric> mat_info, const std::vector<ColParticipants>& col_part) const;
    typedef SolverInterface::ReactionResponse ReactionResponse;
    std::vector<ReactionResponse> processReactions(float delT, Slice < isStaticFlag, lockRotationFlag, Mass, Velocity,
                      AngularVelocity, InertiaDevMass, Position> react_info,
                      const std::vector<MaterialTuple>& mat_info,
                      const std::vector<std::vector<CollisionInfo>>& col_info,
                      const std::vector<ColParticipants>& col_list) const; 
    void applyReactionReseponses(const std::vector<ReactionResponse>& responses, Slice<Velocity, AngularVelocity> velocities, std::vector<ColParticipants> col_list) const;
    //V
    void processNarrowPhase(float delT, ColCompGroup& colliding, const std::vector<ColParticipants>& col_list, const std::vector<SeparatingAxisList>& axes, ThreadPool* tp) const;
    
    void copyResultingVelocities(OwnerSlice<Velocity, AngularVelocity> result_slice, Rigidbody::System& rb_sys) const; 
    void copyResultingTransforms(OwnerSlice<Position, Rotation> result_slice, Transform::System& trans_sys) const; 

    //group contains only objects that can collide and react to collisions
    ColCompGroup createCollidingObjectsGroup(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                                                       Collider::System& col_sys, Material::System& mat_sys) const;
    template<class IntegratorT, class IntegrateeT>
    void integrateAny(float delT, Slice<isStaticFlag, IntegratorT, IntegrateeT> slice) const {
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

    void applyVelocityDrag       (float delT, Slice<Velocity, AirDrag> slice) const;
    void applyAngularVelocityDrag(float delT, Slice<AngularVelocity, AirDrag> slice) const;

    void integrate(float delT, ColCompGroup& group) const;
    
    void rollbackGlobalTransform(Slice<GlobalTransform, LocalTransform> slice) const;
    void updateGlobalTransform(Slice<GlobalTransform, LocalTransform> slice) const;

public :
    // number of physics/collision steps per frame
    int steps = 8U;
    float gravity = 1000.f;

    void update(Transform::System& trans_sys, Rigidbody::System& rb_sys,
                Collider::System& col_sys, Material::System& mat_sys,
                float delT, ThreadPool* thread_pool = nullptr) const;

    // mode used to select bounce when colliding
    eSelectMode bounciness_select = eSelectMode::Min;
    // mode used to select friciton when colliding
    eSelectMode friction_select = eSelectMode::Min;

    template <class T>
    inline void bind() {
        m_solver = std::make_unique<T>();
    }
    // size should be max simulated size
    PhysicsManager() {}
    ~PhysicsManager() {}
};
} // namespace epi
