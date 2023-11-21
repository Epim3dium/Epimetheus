#ifndef EPI_PARTICLES_HPP
#define EPI_PARTICLES_HPP

#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/VertexArray.hpp"
#include "math/math_defs.hpp"
#include <list>
#include <vector>
#include "cell.hpp"
#include "math/math.hpp"
#include "grid.hpp"
struct Particle {
    vec2f pos;
    Cell cell_info;
    vec2f old_pos;
    vec2f force;
    float mass = 1.f;
    static constexpr float radius = 0.7f;
    Particle(vec2f pos, Cell c) : pos(pos), old_pos(pos), force(), cell_info(c) {}
};
struct ParticleManager {
    const size_t width;
    const size_t height;
    typedef std::vector<Particle*> BoxT;

    std::vector< BoxT> boxes;
    std::vector<std::unique_ptr<Particle>> particles;

    ParticleManager(size_t w, size_t h) :  width(w / Particle::radius), height(h / Particle::radius), boxes(width * height) {
    }

    BoxT* boxOF(vec2f pos) {
        int x = floorf(pos.x / Particle::radius);
        int y = floorf(pos.y / Particle::radius);
        if(y >= height || y < 0 || x >= width || x < 0)
            return nullptr;
        return &boxes[y * width + x];
    }
    void add(std::unique_ptr<Particle>&& particle) {
        auto ptr = particle.get();
        particles.push_back(std::move(particle));
        if(boxOF(ptr->pos))
            boxOF(ptr->pos)->push_back(ptr);
    }
    void processCollision(Particle* p, float delT) {
        for(auto dirsx : {0, 1, -1}) {
            for(auto dirsy : {0, 1, -1}) {
                auto* vec = boxOF(p->pos + vec2f(dirsx, dirsy));
                if(!vec)
                    continue;
                for(auto& o : *vec) {
                    if(o == p)
                        continue;
                    auto l = epi::length(p->pos - o->pos);
                    if(isnan(l) || l == 0.f) 
                        continue;
                    if(l < Particle::radius * 2.f) {
                        auto dir = (p->pos - o->pos) / l;
                        p->pos +=  dir * (Particle::radius * 2.f - l) * delT * 0.5f;
                        o->pos += -dir * (Particle::radius * 2.f - l) * delT * 0.5f;
                    }
                }
            }
        }
    }
    std::vector<BoxT*> last_boxes;
    void update(float dt, Grid& grid) {
        // const float iter_count = 8.f;
        // float delT = dt / iter_count;
        //
        // for(int iter = 0; iter < iter_count; iter++) {
        //     for(int i = 0; i < particles.size(); i++) {
        //         auto p = particles[i].get();
        //         p->force += vec2f{0, -100};
        //         auto offset = p->pos - p->old_pos + p->force / p->mass * delT * delT;
        //         p->force = {0, 0};
        //
        //         p->old_pos = p->pos;
        //
        //
        //         p->pos += offset;
        //         if(grid.get(p->pos.x, p->pos.y).type != eCellType::Air) {
        //             grid.set({static_cast<int>(p->old_pos.x), static_cast<int>(p->old_pos.y)}, p->cell_info);
        //             std::swap(particles[i], particles.back());
        //             particles.pop_back();
        //             i--;
        //             continue;
        //             // p->pos = p->old_pos;
        //         }
        //         processCollision(p, delT);
        //     }
        //     for(auto& b : last_boxes)
        //         b->clear();
        //     last_boxes.clear();
        //     for(auto& p : particles) {
        //         auto* box = boxOF(p->pos);
        //         if(!box)
        //             continue;
        //         box->push_back(p.get());
        //         last_boxes.push_back(box);
        //     }
        // }
    }
    void render(sf::RenderTarget& target) {
        // sf::VertexArray arr;
        // arr.setPrimitiveType(sf::PrimitiveType::Points);
        // for(int i = 0; i < particles.size(); i++) {
        //     sf::Vertex v;
        //     v.position = particles[i]->pos;
        //     v.color = particles[i]->cell_info.color;
        //     arr.append(v);
        // }
        // target.draw(arr);

    }
};

#endif //EPI_PARTICLES_HPP
