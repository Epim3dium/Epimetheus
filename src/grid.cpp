#include "grid.hpp"
#include "RNG.h"
#include "particles.hpp"
namespace epi {
const std::unordered_map<eCellType, sf::Color> Cell::g_colors = {
    {eCellType::Bedrock, sf::Color(232, 19, 240)},
    {eCellType::Air, sf::Color(21, 23, 87)},
    {eCellType::Sand, sf::Color(194, 175, 83)},
    {eCellType::Water, sf::Color(52, 108, 130)},
};
RNG rng;

void updateSand(Grid& g, sf::Vector2i vec) {
    int first_dir = rng.Random() > 0.5f ? 1 : -1;
    auto other_coord = vec + vec2i(0, -1);
    if(g.get(other_coord).type != eCellType::Sand && g.get(other_coord).type != eCellType::Bedrock) {
        auto tmp = g.get(vec);
        tmp.isFloating = true;
        g.set(vec, tmp);
        g.swap_at(vec, other_coord);
        return;
    }
    for(auto dir : {vec2i{first_dir, -1}, {-first_dir, -1}}) {
        if(g.get(vec + dir).type != eCellType::Sand && g.get(vec + dir).type != eCellType::Bedrock) {
            g.swap_at(vec, vec + dir);
            return;
        }
    }
    return;
}
void updateNone(Grid& g, sf::Vector2i vec) {
}
void updateWater(Grid& g, sf::Vector2i vec) {
    int first_dir = rng.Random() > 0.5f ? 1 : -1;
    auto other_coord = vec + vec2i(0, -1);
    if(g.get(other_coord).type == eCellType::Air) {
        auto tmp = g.get(vec);
        tmp.isFloating = true;
        g.set(vec, tmp);
        g.swap_at(vec, other_coord);
        return;
    }
    for(auto dir : std::vector<sf::Vector2i>{{0, -1}, {first_dir, -1}, {-first_dir, -1}, {first_dir, 0}, {-first_dir, 0}}) {
        if(g.get(vec + dir).type == eCellType::Air) {
            g.swap_at(vec, vec + dir);
            return;
        }
    }
    return;
}
void Grid::convertFloatingParticles(ParticleManager& manager) {
    for(auto seg : current_segments) {
        for(int y = seg.bottom(); y <= seg.top(); y++) {
            for(int x = seg.left(); x <= seg.right(); x++) {
                if(world[m_idx(x, y)].isFloating) {
                    auto cell = world[m_idx(x, y)];
                    cell.isFloating = false;
                    world[m_idx(x, y)] = Cell(eCellType::Air);
                    manager.add(vec2f(x + ParticleGroup::radius, y + ParticleGroup::radius), cell);
                }
            }
        }
    }
}
const updateFunc_t Cell::g_updates[] = { 
    updateNone,
    updateNone,
    updateSand,
    updateWater,
};

}
