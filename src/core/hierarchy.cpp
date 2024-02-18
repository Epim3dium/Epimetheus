#include "hierarchy.hpp"
#include <map>
namespace epi {
namespace Hierarchy {
    
std::pair<int, std::vector<size_t> > getBFSIndexList(Slice<Entity, Parent> slice) {
    std::vector<size_t> path;
    int max_depth = -1;
    
    std::map<Entity, int> layer_values;
    std::vector<size_t> last_layer_member_index;
    std::vector<size_t> first_layer_member_index;
    path = std::vector<size_t>(slice.size(), -1);
    size_t idx = 0;
    for (auto [id, parent] : slice) {
        int my_layer;
        if (id == parent) {
            my_layer = 0;
        } else {
            my_layer = layer_values.at(parent) + 1;
        }
        layer_values.insert_or_assign(id, my_layer);
        if (my_layer > max_depth) {
            max_depth = my_layer;
            last_layer_member_index.push_back(idx);
            first_layer_member_index.push_back(idx);
        } else {
            path[last_layer_member_index[my_layer]] = idx;
            last_layer_member_index[my_layer] = idx;
        }
        idx++;
    }
    for (int layer = 0; layer < max_depth; layer++) {
        if (path[last_layer_member_index[layer]] == (size_t)-1) {
            path[last_layer_member_index[layer]] =
                first_layer_member_index[layer + 1];
        }
    }
    std::vector<size_t> index_list(path.size());
    index_list[0] = 0;
    size_t cur_path_node = 0U;
    for(int i = 1; i < path.size(); i++) {
        cur_path_node = path[cur_path_node];
        index_list[i] = cur_path_node;
    }
    return {max_depth, index_list};
}
}

}
