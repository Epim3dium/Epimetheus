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
struct StaticParticle {
    vec2f position;
    Cell cell_info;
    vec2f position_old;
    vec2f acc = {};
    float mass;
    size_t id;
    void updatePosition(float delT) {
        auto vel = position - position_old;
        auto vel_n =
            epi::dot(vel, vel) != 0.f ? vel / epi::length(vel) : vec2f();
        constexpr float drag = 0.05f;
        position_old = position;
        position += (vel - vel_n * epi::dot(vel, vel) * drag * delT);

        position += acc / mass * delT * delT;
        acc = {0, -200};
    }
    StaticParticle(vec2f p, Cell c, float m, vec2f a)
        : position(p), position_old(p), cell_info(c), mass(m), acc(a),
          id(getNextId()) {}
};
struct Constraint {
    virtual void update() = 0;
    virtual std::vector<StaticParticle*> getConstraintParticles() = 0;
    virtual ~Constraint() {}
};
struct DistanceConstraint : public Constraint  {
    StaticParticle* p1;
    StaticParticle* p2;
    float len;
    float stretch_damping = 0.f;
    float squeeze_damping = 0.f;
    void update() override {
        auto dir = p1->position - p2->position;
        auto l = length(dir);
        auto diff = len - l;
        auto norm = dir / l;
        auto damping = diff > 0.f ? squeeze_damping : stretch_damping;
        p1->position += norm * diff * (1.f - damping) * 0.5f;
        p2->position -= norm * diff * (1.f - damping) * 0.5f;
    }
    std::vector<StaticParticle*> getConstraintParticles() override {
        return {p1, p2};
    }
    DistanceConstraint(StaticParticle* a, StaticParticle* b, float l)
        : p1(a), p2(b), len(l) {}
    DistanceConstraint(StaticParticle* a, StaticParticle* b) : p1(a), p2(b) {
        len = length(a->position - b->position);
    }
};

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
            if (grid.get(p.x, p.y).type != eCellType::Air) {
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
class ParticleManager {
    size_t width;
    size_t height;
    struct BoxT : public std::vector<vec2f*> {
        bool isUsed = false;
    };

    std::vector<BoxT> boxes;
    std::vector<BoxT*> last_boxes;

    // continuous
    ParticleGroup dynamic_particles;
    // non moving memory
    std::list<StaticParticle> restrained_particles;

    std::vector<std::unique_ptr<Constraint>> restraints;

    bool m_handleCollision(vec2f& p1, float m1, vec2f& p2, float m2) {
        float l = epi::dot(p1 - p2, p1 - p2);
        constexpr float min_r = ParticleGroup::radius * 2.f;
        if (l > min_r * min_r || l == 0.f)
            return false;
        l = sqrt(l);
        vec2f n = (p1 - p2) / l;
        constexpr float response_coef = 0.75f;
        float mag = (min_r - l) * 0.5f * response_coef;

        const float mass_ratio_1 = m1 / (m1 + m2);
        const float mass_ratio_2 = m2 / (m1 + m2);

        p1 += n * mass_ratio_1 * mag;
        p2 += -n * mass_ratio_2 * mag;
        return true;
    }
    void m_processCollisionForBox(BoxT& box) {
        auto adj = getAdjacentBoxes(box);
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
    void m_updatePositions(float delT) {
        dynamic_particles.updatePositions(delT);
        for (auto& p : restrained_particles) {
            p.updatePosition(delT);
        }
    }
    void m_processBoundryConstraintStatic() {
        for (auto& p : restrained_particles) {
            p.position.x = std::clamp<float>(p.position.x, 0, width - 1);
            p.position.y = std::clamp<float>(p.position.y, 0, height - 1);
            p.position_old.x =
                std::clamp<float>(p.position_old.x, 0, width - 1);
            p.position_old.y =
                std::clamp<float>(p.position_old.y, 0, height - 1);
        }
    }
    void m_insertIntoBoxesDynamic() {
        size_t index = 0;
        for (auto& p : dynamic_particles.position) {
            auto* box = boxOf(p);
            if (!box) {
                std::cerr << "out of bounds";
                continue;
            }
            box->push_back(&p);
            if (!box->isUsed)
                last_boxes.push_back(box);
            box->isUsed = true;
            index++;
        }
    }
    void m_insertIntoBoxesStatic() {
        for (auto& particle : restrained_particles) {
            auto& p = particle.position;
            auto* box = boxOf(p);
            if (!box) {
                std::cerr << "out of bounds";
                continue;
            }
            box->push_back(&p);
            if (!box->isUsed)
                last_boxes.push_back(box);
            box->isUsed = true;
        }
    }
    void m_insertIntoBoxes() {
        m_insertIntoBoxesDynamic();
        m_insertIntoBoxesStatic();
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
        for (int i = 0; i != last_boxes.size(); i++) {
            m_processCollisionForBox(*last_boxes[i]);
        }
    }
    void m_processBoundryConstraint() {
        dynamic_particles.processBoundryConstraint({0, 0},
                                                  vec2f(width - 1, height - 1));
        m_processBoundryConstraintStatic();
    }
    std::list<StaticParticle>::iterator
    m_collideWithGridStatic(std::list<StaticParticle>::iterator particle,
                            Grid& grid) {
        auto p = particle->position;
        if (grid.get(p.x, p.y).type != eCellType::Air) {
            vec2i op = (vec2i)particle->position_old;
            if (!(op.x <= 0 || op.y <= 0 || op.x >= grid.width - 1 ||
                  op.y >= grid.height - 1))
                grid.set({static_cast<int>(op.x), static_cast<int>(op.y)},
                         particle->cell_info);
            return restrained_particles.erase(particle);
        }
        return particle;
    }
    void m_processGridCollisions(Grid& grid) {
        dynamic_particles.collideWithGrid(grid);
        for (auto it = restrained_particles.begin();
             it != restrained_particles.end(); it++) {
            it = m_collideWithGridStatic(it, grid);
        }
    }
    void m_processConstraints() {
        for (auto& r : restraints) {
            r->update();
        }
    }

public:
    static constexpr float boxsize = ParticleGroup::radius * 2.f;
    inline size_t size() const {
        return dynamic_particles.size() + restrained_particles.size();
    }
    ParticleManager(size_t w, size_t h)
        : width(w / boxsize), height(h / boxsize), boxes(width * height) {
        std::cerr << width;
    }

    BoxT* boxOf(vec2f pos) {
        int x = floorf(pos.x / boxsize);
        int y = floorf(pos.y / boxsize);
        if (y >= height || y < 0 || x >= width || x < 0)
            return nullptr;
        return &boxes[y * width + x];
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
    void add(vec2f pos, Cell cell, vec2f acc = {}) {
        dynamic_particles.push_back(pos, cell, 1.f, acc);
    }
    void addShape(std::vector<vec2f> shape, Cell cell, vec2f acc = {}) {
        auto center = std::reduce(shape.begin(), shape.end()) / (float)shape.size();
        for(auto& p : shape)
            p -= center;
        sort_clockwise(shape.begin(), shape.end());
        for(auto& p : shape)
            p += center;
        AABBf aabb = AABBfromPolygon(shape);
        auto diameter = ParticleGroup::radius * 2.f;

        std::vector<std::vector<StaticParticle*>> pointer_matrix;

        float tri_h = sqrt(3) * diameter / 2.f;

        size_t yidx = 0;
        for (float y = aabb.min.y; y < aabb.max.y; y += tri_h) {
            pointer_matrix.push_back({});
            auto& last_row = pointer_matrix.back();
            size_t xidx = 0;
            for (float x = aabb.min.x - (yidx % 2) * tri_h * 0.5f; x < aabb.max.x; x += diameter) {
                last_row.push_back(nullptr);
                auto& last_val = last_row.back();
                if (isPointInPolygon(vec2f(x, y), shape)) {
                    restrained_particles.push_back(
                        StaticParticle(vec2f(x, y), cell, 1.f, acc));
                    last_val = &restrained_particles.back();
                }
                xidx++;
            }
            yidx++;
        }
        // creating restraints
        auto checkAndPush = [&](StaticParticle* part1, StaticParticle* part2) {
            if(part1 != nullptr && part2 != nullptr) {
                restraints.push_back(std::make_unique<DistanceConstraint>(
                    DistanceConstraint{part1, part2}));
            }
        };
        for (int ii = 0; ii < pointer_matrix.size(); ii++) {
            const auto& row = pointer_matrix[ii];
            for(int i = 0; i < row.size() - 1; i++) {
                auto* part1 = row[i];
                auto* part2 = row[i + 1];
                checkAndPush(part1, part2);
            }
        }
        for (int ii = 0; ii < pointer_matrix.size() - 1; ii++) {
            const auto& row = pointer_matrix[ii];
            const auto& rowBelow = pointer_matrix[ii + 1];
            for(int i = 0; i < row.size(); i++) {
                auto* part1 = row[i];
                auto* part2 = rowBelow[i];
                checkAndPush(part1, part2);
            }
        }
        for (int ii = 0; ii < pointer_matrix.size() - 1; ii++) {
            const auto& row = pointer_matrix[ii];
            const auto& rowBelow = pointer_matrix[ii + 1];
            for(int i = 0; i < row.size() - 1; i++) {
                auto* part1 = row[i];
                auto* part2 = rowBelow[i + 1];
                checkAndPush(part1, part2);
            }
        }
    }
    void update(float delT, Grid& grid) {
        const size_t steps = 8U;
        float sdelT = delT / static_cast<float>(steps);

        for (int iter = 0; iter < steps; iter++) {

            m_insertIntoBoxes();

            m_processCollisions();

            m_updatePositions(sdelT);

            m_processConstraints();

            m_processBoundryConstraint();

            // m_processGridCollisions(grid);

            for (auto& b : last_boxes) {
                b->isUsed = false;
                b->clear();
            }
            last_boxes.clear();
        }
    }

    void render(sf::RenderTarget& target) {
        sf::VertexArray arr;
        arr.setPrimitiveType(sf::PrimitiveType::Points);
        for (int i = 0; i < dynamic_particles.size(); i++) {
            sf::Vertex v;
            v.position = dynamic_particles.position[i];
            v.color = dynamic_particles.cell_info[i].color;
            arr.append(v);
        }
        for (auto& particle : restrained_particles) {
            sf::Vertex v;
            v.position = particle.position;
            v.color = particle.cell_info.color;
            arr.append(v);
        }
        target.draw(arr);
    }
};

} // namespace epi
#endif // EPI_PARTICLES_HPP
