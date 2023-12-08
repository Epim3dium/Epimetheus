#ifndef EPI_PARTICLES_HPP
#define EPI_PARTICLES_HPP

#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/VertexArray.hpp"
#include "cell.hpp"
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
struct ParticleGroup {
    std::vector<vec2f> position;
    std::vector<Cell> cell_info;
    std::vector<vec2f> position_old;
    std::vector<vec2f> acc;
    std::vector<float> mass;
    std::vector<size_t> ids;
    size_t size() const { return position.size(); }
    void swap(size_t idx1, size_t idx2) {
        if (idx1 == idx2)
            return;
        std::swap(position[idx1], position[idx2]);
        std::swap(position_old[idx1], position_old[idx2]);
        std::swap(cell_info[idx1], cell_info[idx2]);
        std::swap(acc[idx1], acc[idx2]);
        std::swap(mass[idx1], mass[idx2]);
    }
    void pop_back() {
        position.pop_back();
        cell_info.pop_back();
        position_old.pop_back();
        acc.pop_back();
        mass.pop_back();
    }
    void push_back(vec2f p, Cell c, float m, vec2f ac = {}) {
        position.push_back(p);
        position_old.push_back(p);
        cell_info.push_back(c);
        acc.push_back(ac);
        mass.push_back(m);
        ids.push_back(getNextId());
    }
    void updatePositions(float delT) {
        std::vector<vec2f> vel = position;
        std::vector<vec2f> vel_n(vel.size());
        for (size_t i = 0; i < size(); i++) {
            vel[i] -= position_old[i];
            vel_n[i] = epi::dot(vel[i], vel[i]) != 0.f
                           ? vel[i] / epi::length(vel[i])
                           : vec2f();
        }
        constexpr float drag = 0.05f;
        for (size_t i = 0; i < size(); i++) {
            position_old[i] = position[i];
        }
        for (size_t i = 0; i < size(); i++) {
            auto v = vel[i];
            auto vn = vel_n[i];
            position[i] += (v - vn * epi::dot(v, v) * drag * delT);
        }

        for (size_t i = 0; i < size(); i++) {
            position[i] += acc[i] / mass[i] * delT * delT;
        }
        for (auto& a : acc)
            a = {0, -100};
    }
    void collideWithGrid(Grid& grid) {
        for (int i = 0; i < size(); i++) {
            auto p = position[i];
            if (grid.get(p.x, p.y).type != eCellType::Air && !grid.get(p.x, p.y).isFloating) {
                auto op = position_old[i];
                if (!(op.x == 0 || op.y == 0 || op.x == grid.width - 1 ||
                      op.y == grid.height - 1))
                    grid.set({static_cast<int>(op.x), static_cast<int>(op.y)},
                             cell_info[i]);
                swap(i, size() - 1);
                pop_back();
                i--;
                continue;
            }
        }
    }
    void processBoundryConstraint(vec2f min, vec2f max) {
        for (auto& p : position) {
            p.x = std::clamp<float>(p.x, min.x, max.x);
            p.y = std::clamp<float>(p.y, min.y, max.y);
        }
        for (auto& p : position_old) {
            p.x = std::clamp<float>(p.x, min.x, max.x);
            p.y = std::clamp<float>(p.y, min.y, max.y);
        }
    }
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

    static constexpr float boxsize = ParticleGroup::radius * 2.f;

    ParticleCollisionGrid(const float w, const float  h) : width(w / boxsize), height(h / boxsize), boxes(width * height)  {}


    BoxT* boxOf(vec2f pos) {
        int x = floorf(pos.x / boxsize);
        int y = floorf(pos.y / boxsize);
        if (y >= height || y < 0 || x >= width || x < 0)
            return nullptr;
        return &boxes[y * width + x];
    }
    void add(vec2f& p) {
        auto* box = boxOf(p);
        if (!box) {
            std::cerr << "out of bounds";
            return;
        }
        box->push_back(&p);
        if (!box->isUsed)
            used_boxes.push_back(box);
        box->isUsed = true;
    }
    void clear() {
        for (auto& b : used_boxes) {
            b->isUsed = false;
            b->clear();
        }
        used_boxes.clear();
    }
    std::array<BoxT*, 8U> getAdjacentBoxes(BoxT& box) {
        std::array<BoxT*, 8U> result;

        size_t index = &box - &boxes.front();
        int y = index / width;
        int x = index - (y * width);
        size_t cur_idx = 0;
        for (int dy : {1, 0, -1}) {
            for (int dx : {1, 0, -1}) {
                if (dy == 0 && dx == 0)
                    continue;
                if (x + dx < 0 || x + dx >= width || y + dy < 0 ||
                    y + dy >= height) {
                    result[cur_idx] = nullptr;
                } else {
                    result[cur_idx] = &boxes[(y + dy) * width + x + dx];
                }
                cur_idx++;
            }
        }
        assert(cur_idx == 8);

        return result;
    }
};
class ParticleManager {
    float width;
    float height;

    ParticleCollisionGrid part_col_grid;
    // continuous
    ParticleGroup m_particles;
    // non moving memory
    bool m_handleCollision(vec2f& p1, float m1, vec2f& p2, float m2) {
        float l = epi::dot(p1 - p2, p1 - p2);
        constexpr float min_r = ParticleGroup::radius * 2.f;
        if (l > min_r * min_r || l == 0.f)
            return false;
        l = sqrt(l);
        vec2f n = (p1 - p2) / l;
        constexpr float response_coef = 0.75f;
        float mag = (min_r - l) * 0.5f * response_coef;

        const float mass_ratio_1 = m2 / (m1 + m2);
        const float mass_ratio_2 = m1 / (m1 + m2);

        p1 += n * mass_ratio_1 * mag;
        p2 += -n * mass_ratio_2 * mag;
        return true;
    }
    void m_processCollisionForBox(ParticleCollisionGrid::BoxT& box) {
        auto adj = part_col_grid.getAdjacentBoxes(box);
        for (auto p1 : box) {
            for (auto p2 : box) {
                if (p1 == p2)
                    continue;
                m_handleCollision(*p1, 1.f, *p2, 1.f);
            }
        }
        for (auto idx : {0, 1, 2, 3}) {
            if (!adj[idx])
                continue;
            for (auto p1 : box) {
                for (auto p2 : *adj[idx]) {
                    if (p1 == p2)
                        continue;
                    m_handleCollision(*p1, 1.f, *p2, 1.f);
                }
            }
        }
    }
    // void updatePosition(Particle* p, float delT) {
    //     auto vel = p->position - p->position_old;
    //     auto vel_n = epi::dot(vel, vel) != 0.f ? vel / epi::length(vel) :
    //     vec2f(); constexpr float drag = 0.05f; p->position_old = p->position;
    //     p->position += (vel - vel_n * epi::dot(vel, vel) * drag * delT) +
    //     p->acc / p->mass * delT * delT; p->acc = {};
    // }
    void m_insertIntoBoxes() {
        size_t index = 0;
        for (auto& p : m_particles.position) {
            part_col_grid.add(p);
        }
    }
    // ThreadPool tp;
    void m_processCollisions() {
        // tp.dispatch(last_boxes.size() / 2,
        //     [&](size_t begin, size_t end) {
        //         for(int i = begin; i != end; i++) {
        //             m_processCollisionForBox(*last_boxes[i * 2]);
        //         }
        //     });
        // tp.dispatch(last_boxes.size() / 2,
        //     [&](size_t begin, size_t end) {
        //         for(int i = begin; i != std::min(end - 1 + (last_boxes.size()
        //         % 2), end); i++) {
        //             m_processCollisionForBox(*last_boxes[i * 2 + 1]);
        //         }
        //     });
        // tp.waitForCompletion();
        // for(auto& p1 : particles) {
        //     for(auto& p2 : particles) {
        //         if(p1 == p2)
        //             continue;
        //         handleCollision(*p1, *p2);
        //     }
        // }
        for (auto& b : part_col_grid.used_boxes) {
            m_processCollisionForBox(*b);
        }
    }
    void m_processGridCollisions(Grid& grid) {
        m_particles.collideWithGrid(grid);
    }
    struct ParticleInitValues {
        vec2f pos;
        Cell cell;
        vec2f acc;
    };
    std::queue<ParticleInitValues> m_particles_to_add;
    void m_addQueuedParticles() {
        while(m_particles_to_add.size()) {
            auto t = m_particles_to_add.front();
            m_particles.push_back(t.pos, t.cell, 1.f, t.acc);
            m_particles_to_add.pop();
        }
    }

public:
    static constexpr float boxsize = ParticleGroup::radius * 2.f;
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
    void update(float delT, Grid& grid) {
        const size_t steps = 8U;
        float sdelT = delT / static_cast<float>(steps);

        for (int iter = 0; iter < steps; iter++) {

            part_col_grid.clear();

            m_addQueuedParticles();

            m_processGridCollisions(grid);

            m_insertIntoBoxes();

            m_processCollisions();

            m_particles.updatePositions(sdelT);
            
            m_particles.processBoundryConstraint({0, 0}, {width - 1, height - 1});
        }
    }

    void render(sf::RenderTarget& target) {
        sf::VertexArray arr;
        arr.setPrimitiveType(sf::PrimitiveType::Points);
        for (int i = 0; i < m_particles.size(); i++) {
            sf::Vertex v;
            v.position = m_particles.position[i];
            v.color = m_particles.cell_info[i].color;
            arr.append(v);
        }
        target.draw(arr);
    }
};


} // namespace epi
#endif // EPI_PARTICLES_HPP
