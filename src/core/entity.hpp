#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include <_types/_uint64_t.h>
#include <vector>
#include "component_group.hpp"

namespace epi {

template<class Enum>
class ComponentGroup;

enum class eEntity {
    ID_EntityID,
    parentID_EntityID,
    children_vectorEntityID,
};
class Entities : public ComponentGroup<eEntity> {
public:
    typedef uint64_t ID_t;
    typedef std::vector<ID_t> ChildContainer;
private:
    static ID_t m_getNewID();
public:
    static ComponentGroup<eEntity>& get();
    //returns EntityID of entity created
    static ID_t create(ID_t parent = 0);
};

}
#endif // EPI_ENTITY_HPP
