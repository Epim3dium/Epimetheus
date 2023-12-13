#pragma once
#include "SFML/Graphics/Color.hpp"
#include "SFML/System/Vector2.hpp"
#include <unordered_map>
namespace epi {
enum class eCellType {
    Bedrock,
    Stone,
    Air,
    Sand,
    Water,
};
enum class eState {
    Liquid,
    Gas,
    Solid,
    Powder
};
struct CellPropery {
    sf::Color init_color;
    eState state;
    bool isColideable = false;
    float density = 1.f;
};


class Grid;
typedef std::function<void(Grid&, sf::Vector2i)> updateFunc_t;
struct Cell {
    eCellType type;
    sf::Color color;
    size_t last_time_updated = 0;
    bool isFloating = false;

    static const updateFunc_t g_updates[];
    static const CellPropery properties[];

    const CellPropery& getPropery() const {
        return properties[static_cast<size_t>(type)];
    }

    Cell(eCellType t) : type(t) 
    {
        color = getPropery().init_color;
    }

};

}
