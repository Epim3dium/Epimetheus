#include "particles.hpp"

namespace epi {
    void Particles::swap(size_t idx1, size_t idx2) {
        if (idx1 == idx2)
            return;
        std::swap(position[idx1], position[idx2]);
        std::swap(position_old[idx1], position_old[idx2]);
        std::swap(cell_info[idx1], cell_info[idx2]);
        std::swap(acc[idx1], acc[idx2]);
        std::swap(mass[idx1], mass[idx2]);
    }
    void Particles::pop_back() {
        position.pop_back();
        cell_info.pop_back();
        position_old.pop_back();
        acc.pop_back();
        mass.pop_back();
    }
    void Particles::push_back(vec2f p, Cell c, float m, vec2f ac) {
        position.push_back(p);
        position_old.push_back(p);
        cell_info.push_back(c);
        acc.push_back(ac);
        mass.push_back(m);
        ids.push_back(getNextId());
    }
    void Particles::updatePositions(float delT) {
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

    void Particles::collideWithGrid(Grid& grid) {
        auto isInBounds = [&](vec2f p) {
            return (p.x > 0.f && p.y > 0.f && p.x < grid.width && p.y < grid.height);
        };
        for (int i = 0; i < size(); i++) {
            auto p = position[i];
            if (grid.get(p.x, p.y).type == eCellType::Air) {
                continue;
            }

            if(qlen(position_old[i] - position[i]) == 0.f) {
                continue;
            }
            auto n = normal(position_old[i] - position[i]);

            size_t iters = 0;
            static constexpr size_t max_iter_count = 32U;
            while(!isInBounds(p) || grid.get(p.x, p.y).type != eCellType::Air) {
                p += n;
                //p += vec2f(0, 1.f);
                if(max_iter_count == ++iters)
                    break;
            }
            if(isInBounds(p) && iters != max_iter_count) {
                grid.set({static_cast<int>(p.x), static_cast<int>(p.y)},
                         cell_info[i]);
            }
            swap(i, size() - 1);
            pop_back();
            i--;
            continue;
        }
    }
    void Particles::processBoundryConstraint(vec2f min, vec2f max) {
        for (auto& p : position) {
            p.x = std::clamp<float>(p.x, min.x, max.x);
            p.y = std::clamp<float>(p.y, min.y, max.y);
        }
        for (auto& p : position_old) {
            p.x = std::clamp<float>(p.x, min.x, max.x);
            p.y = std::clamp<float>(p.y, min.y, max.y);
        }
    }

    using BoxT = ParticleCollisionGrid::BoxT;

    BoxT* ParticleCollisionGrid::boxOf(vec2f pos) {
        int x = floorf(pos.x / box_size);
        int y = floorf(pos.y / box_size);
        if (y >= height || y < 0 || x >= width || x < 0)
            return nullptr;
        return &boxes[y * width + x];
    }
    void ParticleCollisionGrid::add(vec2f& p) {
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
    void ParticleCollisionGrid::clear() {
        for (auto& b : used_boxes) {
            b->isUsed = false;
            b->clear();
        }
        used_boxes.clear();
    }
    std::array<BoxT*, 8U> ParticleCollisionGrid::getAdjacentBoxes(BoxT& box) {
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
    bool ParticleManager::m_handleCollision(vec2f& p1, float m1, vec2f& p2, float m2) {
        float l = epi::dot(p1 - p2, p1 - p2);
        constexpr float min_r = Particles::radius * 2.f;
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
    void ParticleManager::m_processCollisionForBox(ParticleCollisionGrid::BoxT& box) {
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
    void ParticleManager::m_processObjectCollisions(std::list<DynamicObject>& dynamic_objects) {
        static float constexpr response_coef = 0.1f;
        for(auto& object : dynamic_objects) {
            auto outline = object.getVerticies();
            auto aabb = object.getManifold().collider->getAABB(*object.getManifold().transform);
            for(float y = aabb.min.y; y <= aabb.max.y; y++) {
                for(float x = aabb.min.x; x <= aabb.max.x; x++) {
                    auto particle_positions = queue({x, y});
                    for(auto& part : particle_positions) {
                        if(isOverlappingPointPoly(*part, outline)) {
                            auto diff = findClosestPointOnEdge(*part, outline) - *part;
                            *part += diff * response_coef; 
                        }
                    }
                }
            }
        }
    }
    void ParticleManager::update(float delT, Grid& grid, std::list<DynamicObject>& dyn_objects) {
        const size_t steps = 8U;
        float sdelT = delT / static_cast<float>(steps);

        for (int iter = 0; iter < steps; iter++) {

            m_addQueuedParticles();

            m_processGridCollisions(grid);

            part_col_grid.clear();
            m_insertIntoBoxes();

            m_processCollisions();

            m_processObjectCollisions(dyn_objects);

            m_particles.updatePositions(sdelT);
            
            m_particles.processBoundryConstraint({0, 0}, {width - 1, height - 1});
        }
    }
    void ParticleManager::render(sf::RenderTarget& target) {
        sf::VertexArray arr;
        arr.setPrimitiveType(sf::PrimitiveType::Points);
        for (int i = 0; i < m_particles.size(); i++) {
            sf::Vertex v;
            v.position = m_particles.position[i];
            v.color = m_particles.cell_info[i].color;
            v.color.r *= 1.3f;
            v.color.g *= 1.3f;
            v.color.b *= 1.3f;
            arr.append(v);
        }
        target.draw(arr);
    }
}
