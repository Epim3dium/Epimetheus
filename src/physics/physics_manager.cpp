#include "physics_manager.hpp"
#include "col_utils.hpp"
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
#include <set>
#include <stdexcept>
#include <vector>
#include <thread>
#include <mutex>


namespace epi {
//from 1 to n
//veci communities(n + 1, -1);

Collider* getHead(Collider* col) {
    if(col->parent_collider == col)
        return col;
    std::vector<Collider*> to_speed;
    while(col->parent_collider != col) {
        to_speed.push_back(col);
        col = col->parent_collider;
    }
    for(auto t : to_speed)
        t->parent_collider = col;
    return col;
}

void Merge(Collider* a, Collider* b) {
    a = getHead(a);
    b = getHead(b);
    if(a != b) {
        b->parent_collider = a;
    }
}
bool Friends(Collider* a, Collider* b) {
    return getHead(a) == getHead(b);
}
static bool isDormant(RigidManifold man) {
    return man.collider->isSleeping || man.rigidbody->isStatic;
}

static bool areCompatible(RigidManifold r1, RigidManifold r2) {
    return (!r1.collider->isTrigger || !r2.collider->isTrigger) &&
        !(isDormant(r1) && isDormant(r2)) && 
        (r2.collider->mask.size() == 0 || r1.collider->tag == r2.collider->mask) && 
        (r1.collider->mask.size() == 0 || r2.collider->tag == r1.collider->mask);
}
std::vector<PhysicsManager::ColInfo> PhysicsManager::processBroadPhase() {
    std::vector<PhysicsManager::ColInfo> result;
    std::vector<std::pair<float, RigidManifold>> all;
    for(auto& c : _rigidbodies) {
        auto aabb = c.collider->getAABB(*c.transform);
        all.push_back({aabb.min.x, c});
        all.push_back({aabb.max.x, c});
    }
    std::sort(all.begin(), all.end(),
        [](const std::pair<float, RigidManifold>& p1, const std::pair<float, RigidManifold>& p2) {
            return p1.first < p2.first;
        });
    std::vector<std::pair<RigidManifold, AABB>> open;
    for(auto i : all) {
        auto idx = i.second;
        auto itr = std::find_if(open.begin(), open.end(), 
            [&](const std::pair<RigidManifold, AABB>& p) {
                return p.first == idx;
            });
        if(itr != open.end()) {
            std::swap(*itr, open.back());
            open.pop_back();
            continue;
        }
        auto aabb = i.second.collider->getAABB(*i.second.transform);
        for(auto ii : open) {
            if(isOverlappingAABBAABB(ii.second, aabb))
                result.push_back({idx, ii.first});
        }
        open.push_back({idx, aabb});
    }
    return result;
}
void PhysicsManager::processNarrowPhase(const std::vector<PhysicsManager::ColInfo>& col_list) {
    for(auto ci = col_list.begin(); ci != col_list.end(); ci++) {
        if(!areCompatible(ci->first, ci->second))
            continue;
        auto col_infos = _solver->detect(ci->first.transform, ci->first.collider, ci->second.transform, ci->second.collider);
        for(auto col_info : col_infos) {
            if(!col_info.detected) {
                continue;
            }

            ci->first.collider->notify({*ci->first.collider, *ci->second.collider, col_info});
            col_info.cn *= -1.f;
            ci->second.collider->notify({*ci->second.collider, *ci->first.collider, col_info});
            col_info.cn *= -1.f;

            if(ci->first.collider->isTrigger || ci->second.collider->isTrigger) {
                continue;
            }
            float restitution = selectFrom(ci->first.material->restitution, ci->second.material->restitution, bounciness_select);
            float sfriction = selectFrom(ci->first.material->sfriction, ci->second.material->sfriction, friction_select);
            float dfriction = selectFrom(ci->first.material->dfriction, ci->second.material->dfriction, friction_select);
            _solver->solve(col_info, ci->first, ci->second, restitution, sfriction, dfriction);
            if(!ci->first.rigidbody->isStatic && !ci->second.rigidbody->isStatic)
                Merge(ci->first.collider, ci->second.collider);
        }
    }
}
void PhysicsManager::updateRestraints(float delT) {
    for(auto& r : _restraints)
        r->update(delT);
}
#define DORMANT_MIN_VELOCITY 150.f
#define DORMANT_MIN_ANGULAR_VELOCITY 0.1f
void PhysicsManager::updateRigidObj(RigidManifold& man, float delT) {
    if(man.collider->isTrigger)
        return;
    auto& rb = *man.rigidbody;
    //processing dormants
    if(length(rb.velocity) < DORMANT_MIN_VELOCITY && abs(rb.angular_velocity) < DORMANT_MIN_ANGULAR_VELOCITY) {
        man.collider->time_immobile += delT;
    }else {
        man.collider->time_immobile = 0.f;
    }

    if(isDormant(man)){
        rb.velocity = vec2f();
        rb.angular_velocity = 0.f;
        return;
    }
    if(rb.lockRotation) {
        rb.angular_velocity = 0.f;
        rb.angular_force = 0.f;
    }

    if(!nearlyEqual(qlen(rb.velocity), 0.f))
        rb.velocity -= normal(rb.velocity) * std::clamp(qlen(rb.velocity) * man.material->air_drag, 0.f, length(rb.velocity)) * delT;
    if(!nearlyEqual(rb.angular_velocity, 0.f))
        rb.angular_velocity -= std::copysign(1.f, rb.angular_velocity) * std::clamp(rb.angular_velocity * rb.angular_velocity * man.material->air_drag, 0.f, abs(rb.angular_velocity)) * delT;

    rb.velocity += rb.force / rb.mass * delT;
    rb.angular_velocity += rb.angular_force / man.collider->getInertia(rb.mass) * delT;
    man.transform->setPos(man.transform->getPos() + rb.velocity * delT);
    man.transform->setRot(man.transform->getRot() + rb.angular_velocity * delT);
}
void PhysicsManager::updateRigidbodies(float delT) {
    for(auto r : _rigidbodies) {
        updateRigidObj(r, delT);
    }
}
#define MIN_IMMOBILE_TIME_TO_SLEEP 1.f
void PhysicsManager::processSleeping() {
    std::set<Collider*> parent_colliders_woke;
    for(auto r : _rigidbodies) {
        if(r.collider->isTrigger)
            continue;
        if(r.rigidbody->isStatic)
            continue;
        if(r.collider->time_immobile < MIN_IMMOBILE_TIME_TO_SLEEP) {
            parent_colliders_woke.insert(r.collider->parent_collider);
        }
    }
    for(auto r : _rigidbodies) {
        if(r.collider->isTrigger)
            continue;
        if(r.rigidbody->isStatic)
            continue;

        if(!parent_colliders_woke.contains(r.collider->parent_collider)) {
            r.collider->isSleeping = true;
        }else {
            if(r.collider->isSleeping) {
                r.collider->isSleeping = false;
                r.collider->time_immobile = 0.f;
            }
            r.collider->parent_collider = r.collider;
        }
    }
}
void PhysicsManager::update(float delT) {
    float deltaStep = delT / (float)steps;

    auto col_list = processBroadPhase();
    for(int i = 0; i < steps; i++) {
        updateRestraints(deltaStep);
        updateRigidbodies(deltaStep);
        processNarrowPhase(col_list);
    }

    processSleeping();
    for(auto r : _rigidbodies) {
        r.rigidbody->force = {0.f, 0.f};
        r.rigidbody->angular_force = 0.f;
    }
}
void PhysicsManager::add(RigidManifold man) {
    _rigidbodies.push_back(man);
}
void PhysicsManager::add(Restraint* restraint) {
    _restraints.push_back(restraint);
}
template<class T>
static void unbind_any(const T obj, std::vector<T>& obj_vec) {
    auto itr = obj_vec.begin();
    for(; itr != obj_vec.end(); itr++) {
        if(*itr == obj)
            break;
    }
    if(itr != obj_vec.end()) {
        obj_vec.erase(itr);
    }
}
void PhysicsManager::remove(RigidManifold rb) {
    rb.collider->time_immobile = 0.f;
    rb.collider->parent_collider->time_immobile = 0.f;
    processSleeping();
    unbind_any(rb, _rigidbodies);
}
void PhysicsManager::remove(const Restraint* res) {
    unbind_any<Restraint*>((Restraint*)res, _restraints);
}

}
