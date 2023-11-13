#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include "component_group.hpp"
#include <_types/_uint64_t.h>
#include <vector>

namespace epi {

template <class Enum>
class ComponentGroup;

enum class eEntity {
    parentID_IDt,
    children_chdcontainer,
    name_str,
};
class Entities {
public:
    typedef uint64_t ID_t;
    typedef std::vector<ID_t> ChildContainer;

private:
    ComponentGroup<eEntity>::pointer m_group;
    uint64_t s_ID = 0;
    ID_t m_getNewID();

public:
    void update();

    template <class T>
    inline std::optional<std::reference_wrapper<T>>
    getByID(eEntity variable_name, Entities::ID_t id) {
        return std::move(m_group->getByID<T>(variable_name, id));
    }

    // returns EntityID of entity created
    ID_t create(ID_t parent = 0, std::string name = "");
    void erase(ID_t id);

    void debugPrintHierarchy();
    Entities()
        : m_group(ComponentGroup<eEntity>::Factory()
                      .add<ID_t>(eEntity::parentID_IDt)
                      .add<ChildContainer>(eEntity::children_chdcontainer)
                      .add<std::string>(eEntity::name_str)
                      .create()) {
        // creating world entity
        create(-1);
    }
};

} // namespace epi
#endif // EPI_ENTITY_HPP
