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
struct Particle {
    vec2f position;
    Cell cell_info;
    vec2f position_old;
    vec2f acc;
    float mass = 1.f;
    static constexpr float radius = 0.7f;
    Particle(vec2f pos, Cell c) : position(pos), position_old(pos), acc(), cell_info(c) {}
    void updatePosition(float delT) {
        vec2f offset = position - position_old;
        position_old = position;
        position = position + offset + acc * (delT * delT);
        acc = vec2f();
    }
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

    ParticleManager(size_t w, size_t h) :  width(w / Particle::radius), height(h / Particle::radius), boxes(w * h / (Particle::radius * Particle::radius)) {
    }

    std::array<BoxT*, 4U> getAdjacentBoxes(BoxT& box) {
        std::array<BoxT*, 4U> result;

        size_t index = &box - &boxes.front();
        int y = index / width;
        int x = index - (y * width);
        result[0] = &boxes[y * width + x + 1];
        result[1] = &boxes[y * width + x - 1];
        result[2] = &boxes[(y + 1) * width + x];
        result[3] = &boxes[(y - 1) * width + x];

        return result;
    }
    std::array<BoxT*, 2U> getAdjacentBoxesPositive(BoxT& box) {
        std::array<BoxT*, 2U> result;

        size_t index = &box - &boxes.front();
        int y = index / width;
        int x = index - (y * width);
        result[0] = &boxes[y * width + x + 1];
        result[1] = &boxes[(y + 1) * width + x];
        if(x == width - 1)
            result[0] = nullptr;
        if(y == height - 1)
            result[1] = nullptr;
        return result;
    }

    BoxT* boxOf(vec2f pos) {
        int x = floorf(pos.x / Particle::radius);
        int y = floorf(pos.y / Particle::radius);
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
    void handleCollision(Particle& c1, Particle& c2) {
        float l = epi::length(c1.position - c2.position);
        constexpr float min_r = Particle::radius;
        if(l < min_r * min_r) {
            vec2f n = (c2.position - c1.position) / l;
            constexpr float response_coef = 0.75f;
            float mag = (l - min_r) * 0.5f * response_coef;

            const float mass_ratio_1 = c1.radius / (c1.radius + c2.radius);
            const float mass_ratio_2 = c2.radius / (c1.radius + c2.radius);

            c1.position += n * mass_ratio_1 * mag;
            c2.position += -n* mass_ratio_2 * mag;
        }
    }
    void processCollisionForBox(BoxT& box) {
        auto adj = getAdjacentBoxesPositive(box);
        for(auto p1 : box) {
            for(auto p2 : box) {
                if(p1 == p2)
                    continue;
                handleCollision(*p1, *p2);
            }
        }
        if(adj[0])
            for(auto p1 : box) {
                for(auto p2 : *adj[0]) {
                    if(p1 == p2)
                        continue;
                    handleCollision(*p1, *p2);
                }
            }
        if(adj[1])
            for(auto p1 : box) {
                for(auto p2 : *adj[1]) {
                    if(p1 == p2)
                        continue;
                    handleCollision(*p1, *p2);
                }
            }
    }
    void updatePosition(Particle* p, float delT) {
        auto vel = p->position - p->position_old;
        p->position_old = p->position;
        p->position += vel + p->acc / p->mass * delT * delT;
        p->acc = {};
    }
    void processBoundryConstraint(float minX, float maxX, float minY, float maxY) {
        for(auto& p : particles) {
            if(p->position.x > maxX)
                p->position.x = maxX;
            if(p->position.y > maxY)
                p->position.y = maxY;

            if(p->position.x < minX)
                p->position.x = minX;
            if(p->position.y < minY)
                p->position.y = minY;
        }
    }
    void insertIntoBoxes() {
        for(auto& p : particles) {
            auto* box = boxOf(p->position);
            if(!box)
                continue;
            box->push_back(p.get());
            if(!box->isUsed)
                last_boxes.push_back(box);
            box->isUsed = true;
        }
    }
    void processCollisions() {
        for(auto& b : last_boxes) {
            processCollisionForBox(*b);
        }
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
    void update(float delT, Grid& grid) {
        const size_t iter_count = 8;
        float sdelT = delT / static_cast<float>(iter_count);
        
        for(int iter = 0; iter < iter_count; iter++) {
            for(auto& b : last_boxes) {
                b->isUsed = false;
                b->clear();
            }
            last_boxes.clear();
            processBoundryConstraint(0.f, grid.height, 0.f, grid.width);
            insertIntoBoxes();
            processCollisions();
            updatePositions(sdelT);
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
