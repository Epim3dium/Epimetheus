#ifndef EPI_GRID_HPP
#define EPI_GRID_HPP

#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Image.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/Sprite.hpp"
#include "SFML/Graphics/Texture.hpp"
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>
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
    bool updateBit = false;

    static const std::unordered_map<eCellType, updateFunc_t> g_updates;
    static const std::unordered_map<eCellType, sf::Color> g_colors;

    Cell(eCellType t) : type(t), color(g_colors.at(t)) {}

};

#define NAIIVE
#ifdef NAIIVE
class Grid {
public:
    bool updateBit = false;
    sf::Image img;
    std::vector<Cell> world;
    const size_t width;
    const size_t height;
    Grid(size_t w, size_t h) : width(w), height(h), world(w * h, Cell(eCellType::Air)) 
    {
        img.create(w, h);
    }
    void update() {
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                if(at(x, y).updateBit != this->updateBit)
                    continue;
                auto func = Cell::g_updates.at(world[y * width + x].type);
                if(func) {
                    at(x, y).updateBit = !updateBit;
                    func(*this, {x, y});
                }
            }
        }
        updateBit = !updateBit;
    }
    Cell& at(int x, int y) {
        return world[y * width + x];
    }
    Cell& at(sf::Vector2i v) {
        return world[v.y * width + v.x];
    }
    void swap_at(sf::Vector2i v1, sf::Vector2i v2) {
        at(v1).updateBit = !updateBit;
        at(v2).updateBit = !updateBit;
        std::swap(at(v1.x, v1.y), at(v2.x, v2.y));
    }
    float pixelSize(const sf::Vector2u size) const {
        float scalar = 1.f;
        if(size.x < size.y) {
            scalar = static_cast<float>(size.x) / width;
        }else {
            scalar = static_cast<float>(size.y) / height;
        }
        return scalar;
    }
    void render(sf::RenderTarget& target) {
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                img.setPixel(x, height - y - 1, world[y*width + x].color);
            }
        }
        sf::Texture tex;
        tex.loadFromImage(img);
        sf::Sprite spr;
        auto px_size = pixelSize(target.getSize());
        spr.setScale(px_size, px_size);
        spr.setTexture(tex);
        target.draw(spr);
    }

};
#endif
#endif // EPI_GRID_HPP
