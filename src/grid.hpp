#ifndef EPI_GRID_HPP
#define EPI_GRID_HPP

#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Image.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/Sprite.hpp"
#include "SFML/Graphics/Texture.hpp"
#include "cell.hpp"
#include "physics/collider.hpp"
#include "physics/material.hpp"
#include "physics/physics_manager.hpp"
#include "physics/rigidbody.hpp"
#include "types.hpp"
#include <iostream>
#include <list>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

#define SEGMENT_SIZE 32

namespace epi {
class ParticleManager;

struct SegmentDynamicObject {
    Transform transform;
    Collider collider;
    Rigidbody rigidbody;
    Material material ;
    RigidManifold getManifold() {
        RigidManifold  man;
        man.collider  = &collider;
        man.rigidbody = &rigidbody;
        man.material  = &material;
        man.transform = &transform;
        return man;
    }
    SegmentDynamicObject(ConcavePolygon concave = ConcavePolygon({})) : collider(concave) {
        transform.setPos(concave.getPos());
        rigidbody.isStatic = true;
    }
};
class Grid {
    std::vector<Cell> world;
    struct SegmentT : public AABB {
        bool hasColliderChanged = false;
    };
    std::vector<SegmentT> current_segments;
    std::vector<SegmentT> last_segments;

    inline static bool isCellColliderEligable(const Cell& c) {
        return !c.isFloating && c.getPropery().isColideable;
    }
    void m_updateSegment(SegmentT seg, size_t index);
    std::vector<std::vector<std::pair<vec2f, vec2f>>> m_extractRayGroups(AABB seg);
    std::vector<std::vector<vec2f>> m_extractPolygonPoints(AABB seg);
    ConcavePolygon m_extractPolygon(AABB seg);
public:
    std::list<SegmentDynamicObject> segment_outlines;
    sf::Image img;
    const size_t width;
    const size_t height;
    size_t last_tick_updated = 1;

    inline size_t segmentsWidth() const {
        return width / SEGMENT_SIZE;
    }
    inline size_t segmentsHeight() const {
        return height / SEGMENT_SIZE;
    }
    inline size_t m_idx(int x, int y) const { 
        assert(y < height);
        assert(x < width);
        assert(y >= 0);
        assert(x >= 0);
        return y * width + x; 
    }

    Grid(size_t w, size_t h, PhysicsManager& phy_manager)
        : width(w), height(h), world(w * h, Cell(eCellType::Air)) 
    {
        current_segments = std::vector<SegmentT>(segmentsWidth() * segmentsHeight());
        segment_outlines = std::list<SegmentDynamicObject>(segmentsWidth() * segmentsHeight());
        for(auto& seg : segment_outlines) {
            phy_manager.add(seg.getManifold());
        }
        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                if(x == 0 || y == 0 || x == width - 1 || y == height - 1) {
                    set({x, y}, Cell(eCellType::Bedrock));
                }
            }
        }
        auto itr = segment_outlines.begin();
        for(int y = 0; y < segmentsHeight(); y++) {
            for(int x = 0; x < segmentsWidth(); x++) {
                auto shape = m_extractPolygon({vec2f(x, y) * (float)SEGMENT_SIZE + vec2f(1.f, 1.f), vec2f(x + 1., y + 1.) * (float)SEGMENT_SIZE - vec2f(1.f, 1.f)});
                itr->collider.setShape(shape);
                itr->transform.setPos(shape.getPos());

                current_segments[y * segmentsWidth() + x].max = {-1, -1};
                current_segments[y * segmentsWidth() + x].min = {-1, -1};
                itr++;
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

        size_t idx = 0;
        for(auto& t : last_segments) {
            m_updateSegment(t, idx++);
            t.hasColliderChanged = false;
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
        current_segments[base_y * segmentsWidth() + base_x].hasColliderChanged |= isCellColliderEligable(c);
        current_segments[base_y * segmentsWidth() + base_x].hasColliderChanged |= isCellColliderEligable(world[m_idx(v.x, v.y)]);

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
