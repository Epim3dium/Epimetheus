#include "entity.hpp"
#include "core/component_group.hpp"
namespace epi {

uint64_t Entities::m_getNewID() {
    static uint64_t s_ID = 0;
    return s_ID++;
}

ComponentGroup<eEntity>& Entities::group() {
    static ComponentGroup<eEntity>::pointer s_entities =
        ComponentGroup<eEntity>::Factory()
            //.add<IDtype>(eEntity::ID_EntityID) already managed by ComponentGroup
            .add<IDtype>(eEntity::parentID_EntityID)
            .add<ChildContainer>(eEntity::children_vectorEntityID)
            .create();
    return *s_entities.get(); 
}
uint64_t Entities::create(IDtype parent) {
    auto id = m_getNewID();
    group().push_back(id, parent, std::vector<IDtype>());

    auto arr = group().getByID<ChildContainer>(eEntity::children_vectorEntityID, parent);
    if(arr.has_value()) {
        arr->get().push_back(id);
    }
    return id;
}

} // namespace epi
