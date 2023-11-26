#ifndef EPI_PARTICLES_HPP
#define EPI_PARTICLES_HPP

#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/VertexArray.hpp"
#include "math/math_defs.hpp"
#include <array>
#include <list>
#include <vector>
#include "cell.hpp"
#include "math/math.hpp"
#include "grid.hpp"
#include "multithreading/thread_pool.hpp"
using epi::ThreadPool;

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
    size_t size() const {
        return position.size();
    }
    void swap(size_t idx1, size_t idx2) {
        if(idx1 == idx2)
            return;
        std::swap(position[idx1],     position[idx2]);
        std::swap(position_old[idx1], position_old[idx2]);
        std::swap(cell_info[idx1],    cell_info[idx2]);
        std::swap(acc[idx1],          acc[idx2]);
        std::swap(mass[idx1],         mass[idx2]);
    }
    void pop_back() {
        position    .pop_back();
        cell_info   .pop_back();
        position_old.pop_back();
        acc         .pop_back();
        mass        .pop_back();
    }
    void push_back(vec2f p, Cell c, vec2f ac, float m) {
        position.push_back(p);
        position_old.push_back(p);
        cell_info.push_back(c);
        acc.push_back(ac);
        mass.push_back(m);
        ids.push_back(getNextId());
    }
    static constexpr float radius = 0.5f;
};
class ParticleManager {
public:
    size_t width;
    size_t height;
    struct BoxT : public std::vector<vec2f*> {
        bool isUsed = false;
    };

    std::vector<BoxT> boxes;
    std::vector<BoxT*> last_boxes;
    Particles particles;
    static constexpr float boxsize = Particles::radius * 2.f;

    ParticleManager(size_t w, size_t h) :  width(w / boxsize), height(h / boxsize), boxes(width * height) {
        std::cerr << width;
    }

    std::array<BoxT*, 8U> getAdjacentBoxes(BoxT& box) {
        std::array<BoxT*, 8U> result;

        size_t index = &box - &boxes.front();
        int y = index / width;
        int x = index - (y * width);
        size_t cur_idx = 0;
        for(int dy : {1, 0, -1}) {
            for(int dx : {1, 0, -1}) {
                if(dy == 0 && dx == 0)
                    continue;
                if(x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) {
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

    BoxT* boxOf(vec2f pos) {
        int x = floorf(pos.x / boxsize);
        int y = floorf(pos.y / boxsize);
        if(y >= height || y < 0 || x >= width || x < 0)
            return nullptr;
        return &boxes[y * width + x];
    }
    void add(vec2f pos, Cell cell) {
        particles.push_back(pos, cell, {}, 1.f);
    }
    bool handleCollision(vec2f& p1, float m1, vec2f& p2, float m2) {
        float l = epi::dot(p1 - p2, p1 - p2);
        constexpr float min_r = Particles::radius * 2.f;
        if(l > min_r * min_r || l == 0.f) 
            return false;
        l = sqrt(l);
        vec2f n = (p1 - p2) / l;
        constexpr float response_coef = 0.75f;
        float mag = (min_r - l) * 0.5f * response_coef;

        const float mass_ratio_1 = m1 / (m1 + m2);
        const float mass_ratio_2 = m2 / (m1 + m2);

        p1 += n * mass_ratio_1 * mag;
        p2 += -n* mass_ratio_2 * mag;
        return true;
    }
    void processCollisionForBox(BoxT& box) {
        auto adj = getAdjacentBoxes(box);
        for(auto p1 : box) {
            for(auto p2 : box) {
                if(p1 == p2)
                    continue;
                handleCollision(*p1, 1.f, *p2, 1.f);
            }
        }
        for(auto idx : {0, 1, 2, 3}) {
            if(!adj[idx])
                continue;
            for(auto p1 : box) {
                for(auto p2 : *adj[idx]) {
                    if(p1 == p2)
                        continue;
                    handleCollision(*p1, 1.f, *p2, 1.f);
                }
            }
        }
    }
    // void updatePosition(Particle* p, float delT) {
    //     auto vel = p->position - p->position_old;
    //     auto vel_n = epi::dot(vel, vel) != 0.f ? vel / epi::length(vel) : vec2f();
    //     constexpr float drag = 0.05f;
    //     p->position_old = p->position;
    //     p->position += (vel - vel_n * epi::dot(vel, vel) * drag * delT) + p->acc / p->mass * delT * delT;
    //     p->acc = {};
    // }
    void updatePositions(float delT) {
        auto& p = particles;

        std::vector<vec2f> vel = particles.position;
        std::vector<vec2f> vel_n(vel.size());
        for(size_t i = 0; i < p.size(); i++) {
            vel[i] -= p.position_old[i];
            vel_n[i] = epi::dot(vel[i], vel[i]) != 0.f ? vel[i] / epi::length(vel[i]) : vec2f();
        }
        constexpr float drag = 0.05f;
        for(size_t i = 0; i < p.size(); i++) {
            p.position_old[i] = p.position[i];
        }
        for(size_t i = 0; i < p.size(); i++) {
            auto v = vel[i];
            auto vn = vel_n[i];
            p.position[i] += (v - vn * epi::dot(v, v) * drag * delT);
        }

        for(size_t i = 0; i < p.size(); i++) {
            p.position[i] += p.acc[i] / p.mass[i] * delT * delT;
        }
        for(auto& a : p.acc)
            a = {0, -100};
    }
    void processBoundryConstraint() {
        for(auto& p : particles.position) {
            p.x = std::clamp<float>(p.x, 0, width - 1);
            p.y = std::clamp<float>(p.y, 0, height - 1);
        }
        for(auto& p : particles.position_old) {
            p.x = std::clamp<float>(p.x, 0, width - 1);
            p.y = std::clamp<float>(p.y, 0, height - 1);
        }
    }
    void insertIntoBoxes() {
        size_t index = 0;
        for(auto& p : particles.position) {
            auto* box = boxOf(p);
            if(!box) {
                std::cerr << "out of bounds";
                continue;
            }
            box->push_back(&p);
            if(!box->isUsed)
                last_boxes.push_back(box);
            box->isUsed = true;
            index++;
        }
    }
    ThreadPool tp;
    void processCollisions() {
        if(last_boxes.size() == 0)
            return;
        tp.dispatch(last_boxes.size() / 2, 
            [&](size_t begin, size_t end) {
                for(int i = begin; i != end; i++) {
                    processCollisionForBox(*last_boxes[i * 2]);
                }
            });
        tp.dispatch(last_boxes.size() / 2, 
            [&](size_t begin, size_t end) {
                for(int i = begin; i != std::min(end - 1 + (last_boxes.size() % 2), end); i++) {
                    processCollisionForBox(*last_boxes[i * 2 + 1]);
                }
            });
        tp.waitForCompletion();
        // for(auto& p1 : particles) {
        //     for(auto& p2 : particles) {
        //         if(p1 == p2)
        //             continue;
        //         handleCollision(*p1, *p2);
        //     }
        // }
    }
    void collideWithGrid(Grid& grid) {
        for(int i = 0; i < particles.size(); i++) {
            auto p = particles.position[i];
            if(grid.get(p.x, p.y).type != eCellType::Air) {
                auto op = particles.position_old[i];
                if(!(op.x == 0 || op.y == 0 || op.x == grid.width - 1 || op.y == grid.height - 1))
                    grid.set({static_cast<int>(op.x), static_cast<int>(op.y)}, particles.cell_info[i]);
                particles.swap(i, particles.size() - 1);
                particles.pop_back();
                i--;
                continue;
            }
        }
    }
    void update(float delT, Grid& grid) {
        const size_t iter_count = 1U;
        float sdelT = delT / static_cast<float>(iter_count);
        
        for(int iter = 0; iter < iter_count; iter++) {

            insertIntoBoxes();
            processCollisions();
            updatePositions(sdelT);
            processBoundryConstraint();
            for(auto& b : boxes) {
                b.isUsed = false;
                b.clear();
            }
            last_boxes.clear();
            collideWithGrid(grid);
        }
    }
    
    void render(sf::RenderTarget& target) {
        sf::VertexArray arr;
        arr.setPrimitiveType(sf::PrimitiveType::Points);
        for(int i = 0; i < particles.size(); i++) {
            sf::Vertex v;
            v.position = particles.position[i];
            v.color = particles.cell_info[i].color;
            arr.append(v);
        }
        target.draw(arr);

    }
};

#endif //EPI_PARTICLES_HPP
