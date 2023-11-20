#include "grid.hpp"
#include "RNG.h"
const std::unordered_map<eCellType, sf::Color> Cell::g_colors = {
    {eCellType::Bedrock, sf::Color(232, 19, 240)},
    {eCellType::Air, sf::Color(21, 23, 87)},
    {eCellType::Sand, sf::Color(194, 175, 83)},
    {eCellType::Water, sf::Color(52, 108, 130)},
};
RNG rng;

void updateSand(Grid& g, sf::Vector2i vec) {
    int first_dir = rng.Random() > 0.5f ? 1 : -1;
    for(auto dir : std::vector<sf::Vector2i>{{0, -1}, {first_dir, -1}, {-first_dir, -1}}) {
        if(g.get(vec + dir).type == eCellType::Air) {
            g.swap_at(vec, vec + dir);
            return;
        }
    }
    return;
}
void updateWater(Grid& g, sf::Vector2i vec) {
}
const updateFunc_t Cell::g_updates[] = { 
    updateWater,
    updateWater,
    updateSand,
    updateWater,
};
