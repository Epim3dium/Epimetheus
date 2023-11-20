#ifndef EPI_GRID_HPP
#define EPI_GRID_HPP

#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Image.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/Sprite.hpp"
#include "SFML/Graphics/Texture.hpp"
#include "cell.hpp"
#include "math/aabb.hpp"
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#define SEGMENT_SIZE 32
class Grid {
public:
    sf::Image img;
    std::vector<Cell> world;
    std::vector<AABB> segments[2];
    const size_t width;
    const size_t height;
    size_t last_tick_updated = 1;

    inline size_t segmentsWidth() const {
        return width / SEGMENT_SIZE + 1;
    }
    inline size_t segmentsHeight() const {
        return height / SEGMENT_SIZE + 1;
    }
    inline size_t m_idx(int x, int y) const { return y * width + x; }

    Grid(size_t w, size_t h)
        : width(w), height(h), world(w * h, Cell(eCellType::Air)) 
    {
        segments[0] = std::vector<AABB>(segmentsWidth() * segmentsHeight());
        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                if(x == 0 || y == 0 || x == width - 1 || y == height - 1) {
                    set({x, y}, Cell(eCellType::Bedrock));
                }
            }
        }
        for(int y = 0; y < segmentsHeight(); y++) {
            for(int x = 0; x < segmentsWidth(); x++) {
                segments[0][y * segmentsWidth() + x].max = {-1, -1};
                segments[0][y * segmentsWidth() + x].min = {-1, -1};
            }
        }
        segments[1] = segments[0];
        assert(w == h);
        img.create(w, h);
    }
    void update() {
        std::swap(segments[0], segments[1]);
        for(auto& t : segments[0]) {
            t.min = {0xffffff, 0xffffff};
            t.max = {-1, -1};
        }
        
        auto rng = std::default_random_engine{};
        //std::shuffle(segments[1].begin(), segments[1].end(), rng);

        for(auto& t : segments[1]) {
            if(t.max.x == -1) {
                continue;
            }
            bool yinv = (last_tick_updated % 4 == 0 || last_tick_updated % 3 == 0);
            int yincr = (yinv ? 1 : -1);
            int ybegin = t.bottom() - 1;
            int yend = t.top() + 1;

            for (int y = (yinv ? ybegin : yend); y <= yend && y >= ybegin; y += yincr) {
                int xincr = (last_tick_updated % 2 == 0 ? 1 : -1);
                int xbegin = t.left() - 1;
                int xend = t.right() + 1;
                for(int x = (last_tick_updated % 2 == 0 ? xbegin : xend); x >= xbegin && x <= xend; x += xincr) {
                    if (get(x, y).last_time_updated >= last_tick_updated)
                        continue;
                    world[m_idx(x, y)].last_time_updated = last_tick_updated;
                    auto func = Cell::g_updates[static_cast<size_t>(world[m_idx(x, y)].type)];
                    func(*this, {x, y});
                }
            }
        }
        last_tick_updated++;
    }
    const Cell& get(int x, int y) const { return world[m_idx(x, y)]; }
    const Cell& get(sf::Vector2i v) const { return world[m_idx(v.x, v.y)]; }
    void set(sf::Vector2i v, Cell c) { 
        int base_x = v.x / SEGMENT_SIZE;
        int base_y = v.y / SEGMENT_SIZE;
        segments[0][base_y * segmentsWidth() + base_x].expandToContain(v);

        world[m_idx(v.x, v.y)] = c; 
    }
    void swap_at(sf::Vector2i v1, sf::Vector2i v2) {
        auto tmp = get(v1);
        set(v1, get(v2));
        set(v2, tmp);
    }
    float pixelSize(const sf::Vector2u size) const {
        float scalar = 1.f;
        if (size.x < size.y) {
            scalar = static_cast<float>(size.x) / width;
        } else {
            scalar = static_cast<float>(size.y) / height;
        }
        return scalar;
    }
    void render(sf::RenderTarget& target) {
        for(auto& t : segments[1]) {
            if(t.max.x == -1)
                continue;
            for (int y = t.min.y - 1; y <= t.max.y + 1; y++) {
                for (int x = t.min.x - 1; x <= t.max.x + 1; x++) {
                    img.setPixel(x, height - y - 1, get(x, y).color);
                }
            }
        }
        sf::Texture tex;
        tex.loadFromImage(img);
        sf::Sprite spr;
        spr.setPosition(0, 0);
        auto px_size = pixelSize(target.getSize());
        spr.setScale(px_size, px_size);
        spr.setTexture(tex);
        target.draw(spr);
    }
};
#endif // EPI_GRID_HPP
