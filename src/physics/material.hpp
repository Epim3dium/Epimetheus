#pragma once
#include "core/group.hpp"
#include "math/types.hpp"
#include "templates/primitive_wrapper.hpp"
namespace epi {
// basic class to store physical material properties
namespace Material {
EPI_WRAP_TYPE(PrimitiveWrapper<float>, Restitution);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, StaticFric);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, DynamicFric);
EPI_WRAP_TYPE(PrimitiveWrapper<float>, AirDrag);
struct System : public Group<Restitution, StaticFric, DynamicFric, AirDrag> {
    System() {
        setDefault<Restitution>({0.1f});
        setDefault<StaticFric>({0.8f});
        setDefault<DynamicFric>({0.4f});
        setDefault<AirDrag>({0.1f});
    }
};
};
}
