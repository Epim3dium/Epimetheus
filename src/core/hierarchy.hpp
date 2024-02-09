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

typedef Group<Parent, Children> System;
//returns pair : max_depth, BFS_path
std::pair<int, std::vector<size_t> > getBFSPath(Slice<Entity, Parent> slice);

}
}
#endif //EPI_HIERARCHY_HPP
