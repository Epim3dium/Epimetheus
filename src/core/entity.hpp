#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include "core/group.hpp"
#include <_types/_uint64_t.h>
namespace epi {

enum class eEntity {
    ID_EntityID,
    parentID_EntityID,
    children_vectorEntityID,
};
typedef uint64_t EntityID;
struct Entities {

    static Group<eEntity>& get();
    static uint64_t getNewID();
};

}
#endif // EPI_ENTITY_HPP
