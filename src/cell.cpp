#include "cell.hpp"
#include "grid.hpp"
#include "RNG.h"
namespace epi {
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
const CellPropery Cell::properties[] = {
    {sf::Color(232, 19,  240), eState::Solid,  true},
    {sf::Color(21,  23,  87),  eState::Gas,    false},
    {sf::Color(194, 175, 83),  eState::Powder, true},
    {sf::Color(52,  108, 130), eState::Liquid, false},
};
const updateFunc_t Cell::g_updates[] = { 
    updateNone,
    updateNone,
    updateSand,
    updateWater,
};

}
