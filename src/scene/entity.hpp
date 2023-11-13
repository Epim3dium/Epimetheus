#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include "component_group.hpp"
#include <_types/_uint64_t.h>
#include <vector>

namespace epi {

template <class Enum>
class ComponentGroup;

enum class eEntity {
    ID_IDt,
    parentID_IDt,
    children_chdcontainer,
};
class Entities {
public:
    typedef uint64_t ID_t;
    typedef std::vector<ID_t> ChildContainer;

private:
    ComponentGroup<eEntity>::pointer m_group;
    static ID_t m_getNewID();

public:
    void update();

    template <class T>
    inline std::optional<std::reference_wrapper<T>> getByID(eEntity variable,
                                                            ID_t id) {
        return std::move(m_group->getByID<T>(variable, id));
    }

    // returns EntityID of entity created
    ID_t create(ID_t parent = 0);
    void erase(ID_t id);

    Entities()
        : m_group(ComponentGroup<eEntity>::Factory()
                      .add<ID_t>(eEntity::parentID_IDt)
                      .add<ChildContainer>(eEntity::children_chdcontainer)
                      .create()) {}
};

} // namespace epi
#endif // EPI_ENTITY_HPP
