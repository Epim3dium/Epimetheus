#pragma once
#include "col_utils.hpp"
#include "imgui.h"
#include "transform.hpp"
#include "types.hpp"
#include "math/geometry_func.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <set>

namespace epi {
class Rigidbody;
class Collider;

struct CollisionInfo {
    bool detected;
    vec2f cn;
    std::vector<vec2f> cps;
    float overlap;
};
/*
* \brief Interface Class for creating collider classes , an extension of GAMEOBJECT
*(has prop list and notifies of death)
*( one of 3 key components to collsion simulation)
* virtual functions: 
*   AABB getAABB()
*   eCollisionShape getType()
*   float calcInertia()
*Collider(Transform* trans)
*/
struct ColliderEvent {
    Collider& me;
    Collider& other;
    CollisionInfo info;
};
class Collider : public Signal::Subject<ColliderEvent> {
    float m_inertia_dev_mass = -1.f;
    ConcavePolygon shape;
public:
    Tag tag;
    Tag mask;
    bool isTrigger = false;

    Collider* parent_collider = this;
    float time_immobile = 0.f;
    bool isSleeping = false;

    std::vector<ConvexPolygon> getPolygonShape(Transform& trans) const {
        auto t = shape;
        t.setPos(trans.getPos());
        t.setRot(trans.getRot());
        return t.getPolygons();
    }

    virtual AABB getAABB(Transform& trans) { 
        auto polys = getPolygonShape(trans);
        AABB result = AABB::CreateFromPolygon(polys.front());
        for(auto& p : polys)
            result = result.combine(AABB::CreateFromPolygon(p));
        return result;
    }
    float calcInertia(float mass) {
        float result = 0.f;
        for(const auto& poly : shape.getPolygons()) {
            result += calculateInertia(poly.getModelVertecies(), mass);
        }
        return result; 
    }
    float getInertia(float mass) {
        if(m_inertia_dev_mass == -1.f) {
            m_inertia_dev_mass = calcInertia(mass) / mass;
        }
        return m_inertia_dev_mass * mass;
    }

    Collider() = delete;
    Collider(ConcavePolygon polygons) : shape(polygons)  { 
    }
    virtual ~Collider() {
    }
};

}
