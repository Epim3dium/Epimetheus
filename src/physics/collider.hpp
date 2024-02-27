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
EPI_WRAP_TYPE(std::vector<vec2f>, ShapeModel);
EPI_WRAP_TYPE(std::vector<std::vector<vec2f>>, ShapeTransformedPartitioned);
EPI_WRAP_TYPE(PrimitiveWrapper<bool>, isTriggerFlag);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, InertiaDevMass);
EPI_WRAP_TYPE(epi::Set<std::string>, Tag);
EPI_WRAP_TYPE(epi::Set<std::string>, Mask);

//funtions used
std::vector<std::vector<vec2f>> transformPartitionShape(const std::vector<vec2f>& model_points, const sf::Transform& transform);
float calcInertiaDevMass(const std::vector<std::vector<vec2f>>& model_points);

//system
class System
    : public Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapeTransformedPartitioned> {
    typedef Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapeTransformedPartitioned> MasterClass;
    
public:
    template<class CompTy>
    std::optional<CompTy*> get(Entity owner) {
        static_assert(std::is_same<CompTy, ShapeModel>::value == false);
        return MasterClass::get<CompTy>(owner);
    }
    void setModel(Entity owner, ShapeModel model) {
        auto& model_ref = *MasterClass::get<ShapeModel>(owner).value();
        model_ref = model;
        auto& inertia_ref = *MasterClass::get<InertiaDevMass>(owner).value();
        
        auto dummy_transformed = transformPartitionShape(model, sf::Transform::Identity);
        inertia_ref = Collider::calcInertiaDevMass(dummy_transformed);
    }
    
    void push_back(Entity owner, const std::vector<vec2f>& model,
                   bool isTrigger = false, epi::Set<std::string> tags = {},
                   epi::Set<std::string> masks = {}) {
        auto dummy_transformed = transformPartitionShape(model, sf::Transform::Identity);
        auto inertia = Collider::calcInertiaDevMass(dummy_transformed);
        MasterClass::push_back(owner, {model}, {isTrigger},
                {inertia}, {tags}, {masks}, { ShapeTransformedPartitioned()});
    }
    System() {}
};
void updateCollisionShapes(Slice<Entity, ShapeModel, ShapeTransformedPartitioned> shape_slice, Slice<Transform::GlobalTransform> transform_slice);

};

}
