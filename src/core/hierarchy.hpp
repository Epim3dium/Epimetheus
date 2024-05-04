#ifndef EPI_HIERARCHY_HPP
#define EPI_HIERARCHY_HPP
#include "core/entity.hpp"
#include "core/group.hpp"
#include "templates/primitive_wrapper.hpp"
#include <vector>

namespace epi {
namespace Hierarchy {
EPI_WRAP_TYPE(Entity, Parent);
EPI_WRAP_TYPE(std::vector<Entity>, Children);

struct System : Group<Parent, Children> {
    System() {
        setDefault<Parent>(Entity::invalid());
    }
};
//returns pair : max_depth, BFS_path
std::vector<size_t> getBFSIndexList(OwnerSlice<Parent> slice);
std::vector<size_t> getDFSIndexList(OwnerSlice<Parent, Children> slice);

}
}
#endif //EPI_HIERARCHY_HPP
