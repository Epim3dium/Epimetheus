#pragma once
#include "solver.hpp"
#include "rigidbody.hpp"
#include "restraint.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>
#include <set>


namespace epi {

struct ParticleManager;
/*
 * \brief used to process collision detection and resolution as well as restraints on rigidbodies
 * every Solver, RigidManifold and Trigger have to be bound to be processed, and unbound to stop processing
 * when destroyed all objects will be automaticly unbound
 */
class PhysicsManager {
public:
    enum class eSelectMode {
        Min,
        Max,
        Avg
    };
private:
    template<class T>
    static T selectFrom(T a, T b, eSelectMode mode) {
        switch(mode) {
            case eSelectMode::Avg:
                return (a + b) / 2.f;
            case eSelectMode::Min:
                return std::min(a, b);
            case eSelectMode::Max:
                return std::max(a, b);
        }
    }
    typedef std::pair<RigidManifold, RigidManifold> ColInfo;
    enum class eColType {
        CircCirc,
        PolyPoly,
        CircPoly
    };


    std::vector<RigidManifold> _rigidbodies;
    std::vector<Restraint*> _restraints;

    SolverInterface* _solver = new DefaultSolver();

    std::vector<ColInfo> processBroadPhase();
    void processNarrowPhase(const std::vector<ColInfo>& col_info);
    void processSleeping();

    void updateRigidObj(RigidManifold& man, float delT);

    void updateRigidbodies(float delT);
    void updateRestraints(float delT);

    void processParticles(ParticleManager& pm);
    static AABB getAABBfromRigidbody(RigidManifold man) {
        return man.collider->getAABB(*man.transform);
    }
public:
    //number of physics/collision steps per frame
    size_t steps = 2;

    /*
    * updates all rigidbodies bound applying their velocities and resoving collisions
    * @param pm is particle manager, if bound collisions for all active particles will be resolved(colliding particles will become inactive)
    */
    void update(float delT);

    //mode used to select bounce when colliding
    eSelectMode bounciness_select = eSelectMode::Min;
    //mode used to select friciton when colliding
    eSelectMode friction_select = eSelectMode::Min;


    //used to add any rigidbody
    void add(RigidManifold man);
    //used to add solver that is used to resolve collisions
    inline void bind(SolverInterface* solver) {
        _solver = solver;
    }
    //used to add restraints applied on rigidbodies bound
    void add(Restraint* restraint);
    //removes rigidbody from manager
    void remove(RigidManifold rb);
    //removes restraint from manager
    void remove(const Restraint* restriant);

    //size should be max simulated size
    PhysicsManager(AABB size) {}
    ~PhysicsManager() {}
};
}
