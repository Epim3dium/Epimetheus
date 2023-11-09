#include "entity.hpp"
namespace epi {

uint64_t Entities::getNewID() {
    static uint64_t s_ID = 0;
    return s_ID++;
}

Group<eEntity>& Entities::get() {
    static Group<eEntity>::pointer s_entities =
        Group<eEntity>::Factory()
        .add<EntityID>(eEntity::ID_EntityID)
        .add<EntityID>(eEntity::parentID_EntityID)
        .add< std::vector<EntityID> >(eEntity::children_vectorEntityID)
        .create();
    return *s_entities.get(); 
}

} // namespace epi
