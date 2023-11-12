#include "entity.hpp"
#include "core/component_group.hpp"
namespace epi {

uint64_t Entities::m_getNewID() {
    static uint64_t s_ID = 0;
    return s_ID++;
}

ComponentGroup<eEntity>::pointer Entities::m_group = 
        ComponentGroup<eEntity>::Factory()
            //.add<IDtype>(eEntity::ID_EntityID) already managed by ComponentGroup
            .add<ID_t>(eEntity::parentID_IDt)
            .add<ChildContainer>(eEntity::children_chdcontainer)
            .create();

void Entities::update() {

}
uint64_t Entities::create(ID_t parent) {
    auto id = m_getNewID();
    m_group->push_back(id, parent, std::vector<ID_t>());

    auto arr = m_group->getByID<ChildContainer>(eEntity::children_chdcontainer, parent);
    if(arr.has_value()) {
        arr->get().push_back(id);
    }
    return id;
}
void Entities::erase(ID_t id) {
    auto parent = getByID<ID_t>(eEntity::parentID_IDt, id);
    if(parent.has_value()) {
        auto& vec = getByID<ChildContainer>(eEntity::children_chdcontainer, parent->get()).value().get();
        auto itr = std::find(vec.begin(), vec.end(), id);
        assert(itr != vec.end());
        std::swap(*itr, vec.back());
        vec.pop_back();
    }
    m_group->eraseByID(id);
}

} // namespace epi
