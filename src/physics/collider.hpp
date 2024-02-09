#pragma once
#include "core/group.hpp"
#include "imgui.h"
#include "templates/primitive_wrapper.hpp"
#include "templates/set.hpp"
#include "transform.hpp"
#include "math/types.hpp"
#include "math/geometry_func.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>
#include <set>

namespace epi {

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
    Entity me;
    Entity other;
    CollisionInfo info;
};
namespace Collider {
//variables
struct ShapeModel : public std::vector<vec2f> {
    ShapeModel& operator=(const std::vector<vec2f>& vec) { *this = vec; return *this; }
    ShapeModel& operator=(std::vector<vec2f>&& vec) { *this = vec;      return *this; }
    ShapeModel(const std::vector<vec2f>& vec) : std::vector<vec2f>(vec) {}
    ShapeModel(std::vector<vec2f>&& vec) : std::vector<vec2f>(vec) {}
    ShapeModel(){}
};
struct ShapeTransformedPartitioned : public std::vector<std::vector<vec2f>> {
    ShapeTransformedPartitioned& operator=(const std::vector<std::vector<vec2f>>& vec) { *this = vec; return *this; }
    ShapeTransformedPartitioned& operator=(std::vector<std::vector<vec2f>>&& vec) { *this = vec;      return *this; }
    ShapeTransformedPartitioned(const std::vector<std::vector<vec2f>>& vec) : std::vector<std::vector<vec2f>>(vec) {}
    ShapeTransformedPartitioned(std::vector<std::vector<vec2f>>&& vec) : std::vector<std::vector<vec2f>>(vec) {}
    ShapeTransformedPartitioned(){}
};
struct isTriggerFlag : PrimitiveWrapper<bool> {
    isTriggerFlag(const bool& v) : PrimitiveWrapper<bool>(v) {}
    isTriggerFlag(){}
};
struct InertiaDevMass : PrimitiveWrapper<float> {
    InertiaDevMass(const float& v) : PrimitiveWrapper<float>(v) {}
    InertiaDevMass(){}
};
struct Tag : epi::Set<std::string> {
    Tag& operator=(const epi::Set<std::string>& vec) { *this = vec; return *this; }
    Tag& operator=(epi::Set<std::string>&& vec) { *this = vec;      return *this; }
    Tag(const epi::Set<std::string>& vec) : epi::Set<std::string>(vec) {}
    Tag(epi::Set<std::string>&& vec) : epi::Set<std::string>(vec) {}
    Tag(){}
};
struct Mask : epi::Set<std::string> {
    Mask& operator=(const epi::Set<std::string>& vec) { *this = vec; return *this; }
    Mask& operator=(epi::Set<std::string>&& vec) { *this = vec;      return *this; }
    Mask(const epi::Set<std::string>& vec) : epi::Set<std::string>(vec) {}
    Mask(epi::Set<std::string>&& vec) : epi::Set<std::string>(vec) {}
    Mask(){}
};

//funtions used
std::vector<std::vector<vec2f>> transformPartitionShape(const std::vector<vec2f>& model_points, const Transform& transform);
AABB calcAABB(const std::vector<vec2f>& model_points, const Transform& transform);
float calcInertiaDevMass(const std::vector<std::vector<vec2f>>& model_points);

//system
class System
    : public Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapeTransformedPartitioned> {
    typedef Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapeTransformedPartitioned> MasterClass;
    
public:
    template<class CompTy>
    std::optional<CompTy*> getComponent(Entity owner) {
        static_assert(std::is_same<CompTy, ShapeModel>::value == false);
        return MasterClass::getComponent<CompTy>(owner);
    }
    void setComponent(Entity owner, ShapeModel model) {
        auto& model_ref = *MasterClass::getComponent<ShapeModel>(owner).value();
        model_ref = model;
        auto& inertia_ref = *MasterClass::getComponent<InertiaDevMass>(owner).value();
        
        auto dummy_transformed = transformPartitionShape(model, Transform::Identity);
        inertia_ref = Collider::calcInertiaDevMass(dummy_transformed);
    }
    
    void push_back(Entity owner, const std::vector<vec2f>& model,
                   bool isTrigger = false, epi::Set<std::string> tags = {},
                   epi::Set<std::string> masks = {}) {
        auto dummy_transformed = transformPartitionShape(model, Transform::Identity);
        auto inertia = Collider::calcInertiaDevMass(dummy_transformed);
        MasterClass::push_back(owner, {model}, {isTrigger},
                {inertia}, {tags}, {masks}, { ShapeTransformedPartitioned()});
    }
    System() {}
};
};

}
