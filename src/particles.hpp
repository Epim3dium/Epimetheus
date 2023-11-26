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

struct Particle {
    vec2f position;
    Cell cell_info;
    vec2f position_old;
    vec2f acc;
    float mass = 1.f;
    static constexpr float radius = 0.5f;
    Particle(vec2f pos, Cell c) : position(pos), position_old(pos), acc(), cell_info(c) {}
};
class ParticleManager {
public:
    size_t width;
    size_t height;
    struct BoxT : public std::vector<Particle*> {
        bool isUsed = false;
    };

    std::vector<BoxT> boxes;
    std::vector<BoxT*> last_boxes;
    std::vector<std::unique_ptr<Particle>> particles;
    static constexpr float boxsize = Particle::radius * 2.f;

    ParticleManager(size_t w, size_t h) :  width(w / boxsize), height(h / boxsize), boxes(w * h / (boxsize * boxsize)) {
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
                result[cur_idx++] = &boxes[(y + dy) * width + x + dx];
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
    void add(std::unique_ptr<Particle>&& particle) {
        auto ptr = particle.get();
        particles.push_back(std::move(particle));
        if(boxOf(ptr->position))
            boxOf(ptr->position)->push_back(ptr);
    }
    bool handleCollision(Particle& c1, Particle& c2) {
        float l = epi::length(c1.position - c2.position);
        constexpr float min_r = Particle::radius * 2.f;
        if(l > min_r) 
            return false;
        vec2f n = (c1.position - c2.position) / l;
        constexpr float response_coef = 0.75f;
        float mag = (min_r - l) * 0.5f * response_coef;

        const float mass_ratio_1 = c1.mass / (c1.mass + c2.mass);
        const float mass_ratio_2 = c2.mass / (c1.mass + c2.mass);

        c1.position += n * mass_ratio_1 * mag;
        c2.position += -n* mass_ratio_2 * mag;
        return true;
    }
    void processCollisionForBox(BoxT& box) {
        auto adj = getAdjacentBoxes(box);
        for(auto p1 : box) {
            for(auto p2 : box) {
                if(p1 == p2)
                    continue;
                handleCollision(*p1, *p2);
            }
        }
        for(auto idx : {0, 1, 2, 3}) {
            if(!adj[idx])
                continue;
            for(auto p1 : box) {
                for(auto p2 : *adj[idx]) {
                    if(p1 == p2)
                        continue;
                    handleCollision(*p1, *p2);
                }
            }
        }
    }
    void updatePosition(Particle* p, float delT) {
        auto vel = p->position - p->position_old;
        auto vel_n = epi::dot(vel, vel) != 0.f ? vel / epi::length(vel) : vec2f();
        constexpr float drag = 0.05f;
        p->position_old = p->position;
        p->position += (vel - vel_n * epi::dot(vel, vel) * drag * delT) + p->acc / p->mass * delT * delT;
        p->acc = {};
    }
    void processBoundryConstraint(size_t begin = 0, size_t end = 0xffffff) {
        for(int i = begin; i != std::min(end, particles.size()); i++) {
            auto p = particles[i].get();
            if(p->position.x > (width - 1)   * boxsize)
                p->position.x = (width - 1)  * boxsize;
            if(p->position.y > (height - 1)  * boxsize)
                p->position.y = (height - 1) * boxsize;

            if(p->position.x < 0)
                p->position.x = 0;
            if(p->position.y < 0)
                p->position.y = 0;
        }
    }
    void insertIntoBoxes() {
        for(auto& p : particles) {
            auto* box = boxOf(p->position_old);
            if(!box) {
                std::cerr << "out of bounds";
                continue;
            }
            box->push_back(p.get());
            if(!box->isUsed)
                last_boxes.push_back(box);
            box->isUsed = true;
        }
    }
    void processCollisions() {
        for(auto b : last_boxes) {
            processCollisionForBox(*b);
        }
        // for(auto& p1 : particles) {
        //     for(auto& p2 : particles) {
        //         if(p1 == p2)
        //             continue;
        //         handleCollision(*p1, *p2);
        //     }
        // }
    }
    void updatePositions(float delT) {
        for(int i = 0; i < particles.size(); i++) {
            auto p = particles[i].get();
            p->acc += {0, -100};
            updatePosition(p, delT);
        }
    }
    void collideWithGrid(Grid& grid) {
        for(int i = 0; i < particles.size(); i++) {
            auto p = particles[i].get();
            if(grid.get(p->position.x, p->position.y).type != eCellType::Air) {
                grid.set({static_cast<int>(p->position_old.x), static_cast<int>(p->position_old.y)}, p->cell_info);
                std::swap(particles[i], particles.back());
                particles.pop_back();
                i--;
                continue;
                // p->pos = p->old_pos;
            }
        }
    }
    void update(float delT, Grid& grid, ThreadPool* tp) {
        const size_t iter_count = 8U;
        float sdelT = delT / static_cast<float>(iter_count);
        
        for(int iter = 0; iter < iter_count; iter++) {
            for(auto& b : last_boxes) {
                b->isUsed = false;
                b->clear();
            }
            last_boxes.clear();
            insertIntoBoxes();
            processCollisions();
            updatePositions(sdelT);
            processBoundryConstraint();
            collideWithGrid(grid);
        }
    }
    
    void render(sf::RenderTarget& target) {
        sf::VertexArray arr;
        arr.setPrimitiveType(sf::PrimitiveType::Points);
        for(int i = 0; i < particles.size(); i++) {
            sf::Vertex v;
            v.position = particles[i]->position;
            v.color = particles[i]->cell_info.color;
            arr.append(v);
        }
        target.draw(arr);

    }
};

#endif //EPI_PARTICLES_HPP
