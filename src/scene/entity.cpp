#include "entity.hpp"
#include "debug/log.hpp"
#include "scene/component_group.hpp"
#include <queue>
namespace epi {

uint64_t Entities::m_getNewID() {
    return s_ID++;
}

void Entities::update() {}

uint64_t Entities::create(ID_t parent, std::string name) {
    auto id = m_getNewID();
    auto arr = m_group->getByID<ChildContainer>(eEntity::children_chdcontainer,
                                                parent);
    if(name == "") {
        name = "unnamed entity [" + std::to_string(id) + "]";
    }

    m_group->push_back(id, parent, std::vector<ID_t>(), name);
    EPI_LOG(LogLevel::DEBUG3) << "created " << id;
    if (arr.has_value()) {
        arr->get().push_back(id);
    }
    return id;
}
void Entities::erase(ID_t id) {
    auto parent = getByID<ID_t>(eEntity::parentID_IDt, id);
    if (parent.has_value()) {
        auto& vec = getByID<ChildContainer>(eEntity::children_chdcontainer,
                                            parent->get())
                        .value()
                        .get();
        auto itr = std::find(vec.begin(), vec.end(), id);
        assert(itr != vec.end());
        std::swap(*itr, vec.back());
        vec.pop_back();
    }
    m_group->eraseByID(id);
    EPI_LOG(LogLevel::DEBUG3) << "erased " << id;
}
void Entities::debugPrintHierarchy() {
    std::queue<std::pair<ID_t, size_t>> queue;
    queue.push({0, 0});
    while(queue.size() && m_group->contains(queue.front().first)) {
        auto cur_id = queue.front().first;
        auto cur_depth = queue.front().second;
        queue.pop();
        for(auto i = cur_depth; i--;) {
            std::cout << "\t";
        }
        std::cout << getByID<std::string>(eEntity::name_str, cur_id)->get() << "\n";
        const ChildContainer& ch_vec = getByID<ChildContainer>(eEntity::children_chdcontainer, cur_id)->get();
        for(auto c : ch_vec) {
            queue.push({c, cur_depth + 1});
        }
    }
}

} // namespace epi
