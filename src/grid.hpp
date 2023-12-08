#ifndef EPI_GRID_HPP
#define EPI_GRID_HPP

#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Image.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/Sprite.hpp"
#include "SFML/Graphics/Texture.hpp"
#include "cell.hpp"
#include "types.hpp"
#include <iostream>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#define SEGMENT_SIZE 32
namespace epi {
    class ParticleManager;
class Grid {
    std::vector<Cell> world;
    std::vector<AABB> current_segments;
    std::vector<AABB> last_segments;
    void m_updateSegment(AABB seg);
    std::vector<std::vector<std::pair<vec2f, vec2f>>> m_extractRayGroups(AABB seg);
    std::vector<std::vector<vec2f>> m_extractPolygonPoints(AABB seg);
    ConcavePolygon m_extractPolygon(AABB seg);
public:
    std::vector<ConcavePolygon> segment_outlines;
    sf::Image img;
    const size_t width;
    const size_t height;
    size_t last_tick_updated = 1;

    inline size_t segmentsWidth() const {
        return width / SEGMENT_SIZE + 1;
    }
    inline size_t segmentsHeight() const {
        return height / SEGMENT_SIZE + 1;
    }
    inline size_t m_idx(int x, int y) const { 
        assert(y < height);
        assert(x < width);
        assert(y >= 0);
        assert(x >= 0);
        return y * width + x; 
    }

    Grid(size_t w, size_t h)
        : width(w), height(h), world(w * h, Cell(eCellType::Air)) 
    {
        current_segments = std::vector<AABB>(segmentsWidth() * segmentsHeight());
        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                if(x == 0 || y == 0 || x == width - 1 || y == height - 1) {
                    set({x, y}, Cell(eCellType::Bedrock));
                }
            }
        }
        for(int y = 0; y < segmentsHeight(); y++) {
            for(int x = 0; x < segmentsWidth(); x++) {
                current_segments[y * segmentsWidth() + x].max = {-1, -1};
                current_segments[y * segmentsWidth() + x].min = {-1, -1};
            }
        }
        last_segments = current_segments;
        assert(w == h);
        img.create(w, h, Cell(eCellType::Air).color);
    }
    void update(float delT) {
        std::swap(current_segments, last_segments);
        for(auto& t : current_segments) {
            t.min = {0xffffff, 0xffffff};
            t.max = {-1, -1};
        }
        
        auto rng = std::default_random_engine{};
        //std::shuffle(segments[1].begin(), segments[1].end(), rng);

        for(auto& t : last_segments) {
            m_updateSegment(t);
        }
        last_tick_updated++;
    }
    void convertFloatingParticles(ParticleManager& manager);

    const Cell& get(int x, int y) const {
        return world[m_idx(x, y)]; 
    }
    const Cell& get(sf::Vector2i v) const { return get(v.x, v.y); }
    void set(sf::Vector2i v, Cell c) { 
        int base_x = v.x / SEGMENT_SIZE;
        int base_y = v.y / SEGMENT_SIZE;
        current_segments[base_y * segmentsWidth() + base_x].expandToContain(static_cast<vec2f>(v));

        world[m_idx(v.x, v.y)] = c; 
    }
    void swap_at(sf::Vector2i v1, sf::Vector2i v2) {
        auto tmp = get(v1);
        set(v1, get(v2));
        set(v2, tmp);
    }
    void render(sf::RenderTarget& target) {
        for(auto& t : last_segments) {
            if(t.max.x == -1)
                continue;
            for (int y = t.min.y - 1; y <= t.max.y + 1; y++) {
                for (int x = t.min.x - 1; x <= t.max.x + 1; x++) {
                    auto color = get(x, y).color;
                    if(get(x, y).isFloating)
                        color = sf::Color::Red;
                    img.setPixel(x, y, color);
                }
            }
        }
        sf::Texture tex;
        tex.loadFromImage(img);
        sf::Sprite spr;
        spr.setPosition(0, 0);
        spr.setTexture(tex);
        target.draw(spr);
    }
};

}
#endif // EPI_GRID_HPP
