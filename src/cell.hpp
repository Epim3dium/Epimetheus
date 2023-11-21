#pragma once
#include "SFML/Graphics/Color.hpp"
#include "SFML/System/Vector2.hpp"
#include <unordered_map>
enum class eCellType {
    Bedrock,
    Air,
    Sand,
    Water,
};


class Grid;
typedef std::function<void(Grid&, sf::Vector2i)> updateFunc_t;
struct Cell {
    eCellType type;
    sf::Color color;
    size_t last_time_updated = 0;

    static const updateFunc_t g_updates[];
    static const std::unordered_map<eCellType, sf::Color> g_colors;

    Cell(eCellType t) : type(t), color(g_colors.at(t)) {}

};
