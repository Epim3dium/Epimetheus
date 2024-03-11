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
    vec2f contact_normal;
    vec2f contact_point;
    float overlap;
};
struct ColliderEvent {
    Entity me;
    Entity other;
    CollisionInfo info;
};
namespace Collider {
//variables
EPI_WRAP_TYPE(std::vector<vec2f>, ShapeModel);
EPI_WRAP_TYPE(std::vector<std::vector<vec2f>>, ShapePartitioned);
EPI_WRAP_TYPE(std::vector<std::vector<vec2f>>, ShapeTransformedPartitioned);
EPI_WRAP_TYPE(PrimitiveWrapper<bool>, isTriggerFlag);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, InertiaDevMass);
EPI_WRAP_TYPE(epi::Set<std::string>, Tag);
EPI_WRAP_TYPE(epi::Set<std::string>, Mask);

//funtions used
std::vector<std::vector<vec2f>> partitionShape(std::vector<vec2f> model_points);
float calcInertiaDevMass(const std::vector<std::vector<vec2f>>& model_points);

//system
class System
    : public Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapePartitioned, ShapeTransformedPartitioned> {
    typedef Group<ShapeModel, isTriggerFlag, InertiaDevMass, Tag, Mask, ShapePartitioned, ShapeTransformedPartitioned> MasterClass;
    
public:
    template<class CompTy>
    std::optional<CompTy*> get(Entity owner) {
        static_assert(std::is_same<CompTy, ShapeModel>::value == false);
        return MasterClass::get<CompTy>(owner);
    }
    void setModel(Entity owner, ShapeModel model) {
        auto& model_ref = MasterClass::get<ShapeModel>(owner);
        model_ref = model;
        
        auto& inertia_ref = MasterClass::get<InertiaDevMass>(owner);
        
        auto partitioned = partitionShape(model);
        inertia_ref = Collider::calcInertiaDevMass(partitioned);
    }
    
    void push_back(Entity owner, const std::vector<vec2f>& model,
                   bool isTrigger = false, epi::Set<std::string> tags = {},
                   epi::Set<std::string> masks = {}) {
        auto partitioned = partitionShape(model);
        auto inertia = Collider::calcInertiaDevMass(partitioned);
        MasterClass::push_back(owner, {model}, {isTrigger},
                {inertia}, {tags}, {masks}, {ShapePartitioned()}, { ShapeTransformedPartitioned()});
    }
    System() {
        setDefault<isTriggerFlag>({false});
        setDefault<InertiaDevMass>({1.f});
    }
};
void calcParitionedShapes(Slice<ShapeModel, ShapePartitioned> shape_slice);
void updatePartitionedTransformedShapes(OwnerSlice<ShapePartitioned, ShapeTransformedPartitioned> shape_slice, Slice<Transform::GlobalTransform> transform_slice);

};

}
