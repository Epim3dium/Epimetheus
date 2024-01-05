#ifndef EPI_PARTICLES_HPP
#define EPI_PARTICLES_HPP


#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/VertexArray.hpp"
#include "cell.hpp"
#include "dynamic_object.hpp"
#include "grid.hpp"
#include "math/geometry_func.hpp"
#include "math/math.hpp"
#include "math/math_defs.hpp"
#include "multithreading/thread_pool.hpp"
#include <array>
#include <list>
#include <vector>
using epi::ThreadPool;

namespace epi {
inline static size_t getNextId() {
    static size_t s_ID = 1;
    return s_ID++;
}
struct Particles {
    std::vector<vec2f> position;
    std::vector<Cell> cell_info;
    std::vector<vec2f> position_old;
    std::vector<vec2f> acc;
    std::vector<float> mass;
    std::vector<size_t> ids;
    size_t size() const { return position.size(); }
    void swap(size_t idx1, size_t idx2) ;
    void pop_back() ;
    void push_back(vec2f p, Cell c, float m, vec2f ac = {}) ;
    void updatePositions(float delT) ;
    void collideWithGrid(Grid& grid) ;
    void processBoundryConstraint(vec2f min, vec2f max);
    static constexpr float radius = 0.5f;
};
struct ParticleCollisionGrid {
    struct BoxT : public std::vector<vec2f*> {
        bool isUsed = false;
    };
    const size_t height;
    const size_t width;

    std::vector<BoxT> boxes;
    std::vector<BoxT*> used_boxes;

    static constexpr float box_size = Particles::radius * 2.f;

    ParticleCollisionGrid(const float w, const float  h) : width(w / box_size), height(h / box_size), boxes(width * height)  {}


    BoxT* boxOf(vec2f pos);
    void add(vec2f& p);
    void clear();
    std::array<BoxT*, 8U> getAdjacentBoxes(BoxT& box);
};
class ParticleManager {
    float width;
    float height;

    ParticleCollisionGrid part_col_grid;
    // continuous
    Particles m_particles;
    // non moving memory
    bool m_handleCollision(vec2f& p1, float m1, vec2f& p2, float m2);
    void m_processCollisionForBox(ParticleCollisionGrid::BoxT& box);
    // void updatePosition(Particle* p, float delT) {
    //     auto vel = p->position - p->position_old;
    //     auto vel_n = epi::dot(vel, vel) != 0.f ? vel / epi::length(vel) :
    //     vec2f(); constexpr float drag = 0.05f; p->position_old = p->position;
    //     p->position += (vel - vel_n * epi::dot(vel, vel) * drag * delT) +
    //     p->acc / p->mass * delT * delT; p->acc = {};
    // }
    inline void m_insertIntoBoxes() {
        for (auto& p : m_particles.position) {
            part_col_grid.add(p);
        }
    }
    // ThreadPool tp;
    inline void m_processCollisions() {
        for (auto& b : part_col_grid.used_boxes) {
            m_processCollisionForBox(*b);
        }
    }
    inline void m_processGridCollisions(Grid& grid) {
        m_particles.collideWithGrid(grid);
    }
    void m_processObjectCollisions(std::list<DynamicObject>& dynamic_objects);
    struct ParticleInitValues {
        vec2f pos;
        Cell cell;
        vec2f acc;
    };
    std::queue<ParticleInitValues> m_particles_to_add;
    inline void m_addQueuedParticles() {
        while(m_particles_to_add.size()) {
            const auto& t = m_particles_to_add.front();
            m_particles.push_back(t.pos, t.cell, 1.f, t.acc);
            m_particles_to_add.pop();
        }
    }

public:
    static constexpr float boxsize = Particles::radius * 2.f;
    inline size_t size() const {
        return m_particles.size();
    }
    ParticleManager(float w, float h)
        : width(w), height(h), part_col_grid(width, height) {
    }

    void add(vec2f pos, Cell cell, vec2f acc = {}) {
        m_particles_to_add.push({pos, cell, acc});
    }
    std::vector<vec2f*> queue(vec2f pos) {
        if(part_col_grid.boxOf(pos))
            return *part_col_grid.boxOf(pos);
        return {};
    }
    void update(float delT, Grid& grid, std::list<DynamicObject>& dyn_objects);

    void render(sf::RenderTarget& target);
};


} // namespace epi
#endif // EPI_PARTICLES_HPP
