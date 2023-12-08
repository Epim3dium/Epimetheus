#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "grid.hpp"
#include "imgui.h"
#include "particles.hpp"
#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "physics/physics_manager.hpp"
#include "timer.h"
#include "utils/time.hpp"


using namespace epi;

struct DynamicObject {
    Transform transform;
    Collider collider;
    Rigidbody rigidbody;
    Material material;
    RigidManifold getManifold() {
        RigidManifold man;
        man.transform =&transform;
        man.rigidbody =&rigidbody;
        man.material = &material;
        man.collider = &collider;
        return man;
    }
    DynamicObject(ConcavePolygon poly) : collider(poly) {
        transform.setPos(poly.getPos());
    }
    static DynamicObject create(ConvexPolygon poly) {
        return DynamicObject(ConcavePolygon({poly}));
    }
    static DynamicObject create(std::vector<vec2f> points) {
        auto triangles = toTriangles(points);
        auto polygons = toBiggestConvexPolygons(triangles);
        ConcavePolygon concave_poly(polygons);
        return DynamicObject(concave_poly);
    }
};

#define GRID_DOWNSCALE 2
class Demo : public App {
public:
    PhysicsManager physics_manager;
    std::list<DynamicObject> dynamic_objects;

    Grid grid;

    ParticleManager particle_manager;

    sf::RenderTexture downscaled;

    struct {
        std::vector<vec2f> rigid_construct_points;
    }opt;
    float pixelSize(const sf::Vector2u size) const {
        float scalar = 1.f;
        if (size.x < size.y) {
            scalar = static_cast<float>(size.x) / (width >> GRID_DOWNSCALE);
        } else {
            scalar = static_cast<float>(size.y) / (height >> GRID_DOWNSCALE);
        }
        return scalar;
    }
    bool onSetup() override { 
        downscaled.create(width >> GRID_DOWNSCALE, height >> GRID_DOWNSCALE);
        physics_manager.bounciness_select = PhysicsManager::eSelectMode::Max;
        physics_manager.friction_select = PhysicsManager::eSelectMode::Max;
        event_handler.addCallback(sf::Event::KeyPressed, 
            [&](const sf::Event& e) {
                if(e.key.code == sf::Keyboard::Enter) {
                    auto concave = DynamicObject::create(opt.rigid_construct_points);
                    dynamic_objects.push_back(concave);
                    dynamic_objects.back().collider.parent_collider = &dynamic_objects.back().collider;
                    if(e.key.shift)
                        dynamic_objects.back().rigidbody.isStatic = true;
                    physics_manager.add(dynamic_objects.back().getManifold());
                }
            });
        event_handler.addCallback(sf::Event::KeyPressed, 
            [&](const sf::Event& e) {
                if(e.key.code == sf::Keyboard::R) {
                    opt.rigid_construct_points.clear();
                }
            });
        event_handler.addCallback(sf::Event::MouseButtonPressed, 
            [&](const sf::Event& e) {
                if(e.mouseButton.button == sf::Mouse::Right) {
                    vec2f mouse_pos = (vec2f)getMousePos();
                    auto px_size = pixelSize(getSize());
                    mouse_pos.x /= px_size;
                    mouse_pos.y /= px_size;
                    mouse_pos.y = grid.height - mouse_pos.y - 1;
                    opt.rigid_construct_points.push_back(mouse_pos);
                }
            });
        return true;
    }
    void onUpdate() override {
        int brush_size = 10;
        sf::Keyboard::Key cur_key = sf::Keyboard::F12;
        bool isDrawing = false;
        auto drawMaterial = [&](sf::Keyboard::Key key, eCellType type) {
            if(!sf::Keyboard::isKeyPressed(key))
                return;
            auto mouse_pos = getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            Cell c(type);
            for(int i = -brush_size / 2; i < brush_size / 2; i++) 
                for(int ii = -brush_size / 2; ii < brush_size / 2; ii++) 
                    grid.set({(int)mouse_pos.x + i , static_cast<int>(grid.height - (int)mouse_pos.y - 1 + ii)}, c);
        };
        drawMaterial(sf::Keyboard::S, eCellType::Sand);
        drawMaterial(sf::Keyboard::Z, eCellType::Bedrock);
        drawMaterial(sf::Keyboard::X, eCellType::Air);
        static vec2i last_mouse_pos = getMousePos();
        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            vec2f mouse_pos = (vec2f)getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            mouse_pos.y = grid.height - mouse_pos.y - 1;
            const float spawn_size = 5.0f;
            auto force = vec2f(getMousePos() - last_mouse_pos) * 2000.f;
            force.y *= -1;
            static int ticks  = 0;
            ticks++;
            bool canSpawn = true;
            for(auto dir : {vec2f{0, 0}, {0, -1}, {0, 1}, {1, 0}, {-1, 0}, {1, 1}, {-1, -1}}) {
                if(particle_manager.queue(mouse_pos + dir * ParticleGroup::radius).size() != 0) {
                    canSpawn = false;
                }
            }
            if(canSpawn) {
                for(float y = mouse_pos.y - spawn_size / 2; y < mouse_pos.y + spawn_size / 2; y += ParticleGroup::radius * 2.f) {
                    for(float x = mouse_pos.x - spawn_size / 2; x < mouse_pos.x + spawn_size / 2; x += ParticleGroup::radius * 2.f) {
                        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                            particle_manager.add({x, y}, Cell(eCellType::Water), force);
                        }
                    }
                }
            }
        }
        last_mouse_pos = getMousePos();
        grid.update(Time::deltaTime());
        // grid.convertFloatingParticles(particle_manager);
        particle_manager.update(Time::deltaTime(), grid);
        for(auto& object : dynamic_objects)
            object.rigidbody.addForce(vec2f{0, -100} * object.rigidbody.mass * static_cast<float>(Time::deltaTime()));
        for(auto itr = dynamic_objects.begin(); itr != dynamic_objects.end(); itr++) {
            auto& object = *itr;
            if(!isOverlappingAABBAABB(object.collider.getAABB(object.transform), AABB::CreateMinMax(vec2f{0, 0}, vec2f(grid.width, grid.height)))) {
                physics_manager.remove(object.getManifold());
                itr = dynamic_objects.erase(itr);
            }
        }
        physics_manager.update(Time::deltaTime());
    }
    void onRender(sf::RenderWindow& window) override {
        grid.render(downscaled);
        particle_manager.render(downscaled);

        for(auto p : opt.rigid_construct_points) {
            sf::Vertex vert;
            vert.position = p;
            vert.color = sf::Color::Red;
            downscaled.draw(&vert, 1U, sf::Points);
        }
        // for(const auto& object : dynamic_objects) {
        //     for(auto poly : object.collider.getPolygonShape(object.transform)) {
        //         drawFill(downscaled, poly, sf::Color::White);
        //     }
        // }

        sf::Sprite spr;
        spr.setPosition(0, 0);
        spr.setTexture(downscaled.getTexture());
        float scalar = pixelSize(window.getSize());
        spr.setScale(scalar, scalar);
        window.draw(spr);

        auto getScreenPos = [&](vec2f grid_pos) {
            auto px_size = pixelSize(window.getSize());
            return vec2f(grid_pos.x * px_size, window.getSize().y - grid_pos.y * px_size);
        };
        for(const auto& object : dynamic_objects) {
            for(auto poly : object.collider.getPolygonShape(object.transform)) {
                auto points = poly.getVertecies();

                sf::Vertex vert[2];
                vert[0].color = vert[1].color = sf::Color::Cyan;
                vert[0].position = getScreenPos(points.back());
                for(auto& p : points) {
                    vert[1].position = getScreenPos(p);
                    window.draw(vert, 2U, sf::Lines);
                    vert[0] = vert[1];
                }
            }
        }
        for(const auto& concave : grid.segment_outlines) {
            for(auto poly : concave.getPolygons()) {
                auto points = poly.getVertecies();

                sf::Vertex vert[2];
                vert[0].color = vert[1].color = sf::Color::Cyan;
                vert[0].position = getScreenPos(points.back());
                for(auto& p : points) {
                    vert[1].position = getScreenPos(p);
                    window.draw(vert, 2U, sf::Lines);
                    vert[0] = vert[1];
                }
            }
        }

        ImGui::Begin("options");
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
        1000.0/double(ImGui::GetIO().Framerate), double(ImGui::GetIO().Framerate));
        ImGui::Text("particle count: %zu", particle_manager.size());
        ImGui::Text("dynamic object count: %zu", dynamic_objects.size());
        ImGui::End();
    }
    Demo(size_t w, size_t h) : App(w, h, "demo"), grid(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE), particle_manager(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE) {}
    ~Demo() { std::cerr << "demo destroyed\n"; }
};

int main(int argc, char** argv)
{
    {
        auto d = new Demo(1 << 10, 1 << 10);
        d->run();
        delete d;
    }
}
