#include "cell.hpp"
#include "grid.hpp"
#include "RNG.h"
namespace epi {
RNG rng;

void updateSand(Grid& g, sf::Vector2i vec) {
    int first_dir = rng.Random() > 0.5f ? 1 : -1;
    auto other_coord = vec + vec2i(0, -1);
    auto checkSwappable = [&](vec2i coord) {
        return g.get(coord).getPropery().state != eState::Solid && g.get(coord).getPropery().state != eState::Powder;
    };
    if(checkSwappable(other_coord)) {
        auto tmp = g.get(vec);
        tmp.isFloating = true;
        g.set(vec, tmp);
        if(g.get(other_coord).getPropery().state != eState::Gas) {
            g.swap_at(vec, other_coord);
        }
        return;
    }
    for(auto dir : {vec2i{first_dir, -1}, {-first_dir, -1}}) {
        if(checkSwappable(vec + dir)) {
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
        //g.swap_at(vec, other_coord);
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
updateFunc_t Cell::g_updates[static_cast<size_t>(eCellType::TYPE_COUNT)];
CellPropery Cell::properties[static_cast<size_t>(eCellType::TYPE_COUNT)];
bool Cell::initializeProperties() {
    Cell::g_updates[static_cast<size_t>(eCellType::Bedrock)] = updateNone;
    Cell::g_updates[static_cast<size_t>(eCellType::Stone)]   = updateNone;
    Cell::g_updates[static_cast<size_t>(eCellType::Air)]     = updateNone;
    Cell::g_updates[static_cast<size_t>(eCellType::Sand)]    = updateSand;
    Cell::g_updates[static_cast<size_t>(eCellType::Water)]   = updateWater;
    Cell::g_updates[static_cast<size_t>(eCellType::Barrier)] = updateNone;

    Cell::properties[static_cast<size_t>(eCellType::Bedrock)] =  {sf::Color(232, 19,  240), eState::Solid,  true};
    Cell::properties[static_cast<size_t>(eCellType::Stone)]   =  {sf::Color(127, 127, 127), eState::Solid,  true};
    Cell::properties[static_cast<size_t>(eCellType::Air)]     =  {sf::Color(21,  23,  87),  eState::Gas,    false};
    Cell::properties[static_cast<size_t>(eCellType::Sand)]    =  {sf::Color(194, 175, 83),  eState::Powder, true};
    Cell::properties[static_cast<size_t>(eCellType::Water)]   =  {sf::Color(52,  108, 130), eState::Liquid, false};
    Cell::properties[static_cast<size_t>(eCellType::Barrier)] =  {sf::Color(155, 155, 0),   eState::Solid,  false};

    return true;
}
bool Cell::hasInitializedProperties = initializeProperties();

}
