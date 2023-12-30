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
    //rigidbody stuff
    Transform transform;
    Collider collider;
    Rigidbody rigidbody;
    Material material;

    //texture stuff
    sf::Texture tex;
    vec2f tex_size;
    vec2f tex_offset;

    //collision with particles
    std::vector<vec2f> model_points;
    std::vector<vec2f> getVerticies() const {
        std::vector<vec2f> verticies;
    }


    RigidManifold getManifold() {
        RigidManifold man;
        man.transform =&transform;
        man.rigidbody =&rigidbody;
        man.material = &material;
        man.collider = &collider;
        return man;
    }
    void m_carveShape(ConcavePolygon shape, Grid& grid) {
        AABB bounding_box = AABB::CreateFromPolygon(shape.getPolygons().front());
        for(auto& p : shape.getPolygons())
            bounding_box = bounding_box.combine(AABB::CreateFromPolygon(p));
        sf::Image rend_tex;

        rend_tex.create(bounding_box.size().x, bounding_box.size().y, sf::Color::Transparent);
        tex_size = bounding_box.size();
        tex_offset = -shape.getPos() + bounding_box.center();

        for(int y = bounding_box.bottom(); y < bounding_box.top(); y++) {
            for(int x = bounding_box.left(); x < bounding_box.right(); x++) {
                bool isInside = false;
                for(auto convex : shape.getPolygons()) {
                    if(isOverlappingPointPoly(vec2f(x, y) + vec2f(0.5f, 0.5f), convex.getVertecies())) {
                        isInside = true;
                    }
                }
                if(!isInside) {
                    continue;
                }
                sf::Vertex vert;
                auto cell = grid.get(x, y);
                if(cell.type == eCellType::Air)
                    continue;
                auto coord = vec2f(x, y) - bounding_box.min;
                rend_tex.setPixel(coord.x, coord.y, cell.color);
                grid.set({x, y}, Cell(eCellType::Air));
            }
        }
        tex.loadFromImage(rend_tex);

    }
    DynamicObject(ConcavePolygon shape, Grid& grid) : collider(shape) {
        m_carveShape(shape, grid);
        transform.setPos(shape.getPos());
    }
    static void sortClockWise(std::vector<vec2f>& points) {
        int sign = 0;
        for(int i = 0; i < points.size(); i++) {
            auto first = points[i];
            auto mid = points[(i + 1) % points.size()];
            auto last = points[(i + 2) % points.size()];
            float angle = angleAround(first, mid, last);
            sign += angle > 0.f ? 1 : -1;
        }
        if(sign < 0) {
            std::reverse(points.begin(), points.end());
        }
    }
    static DynamicObject create(std::vector<vec2f> points, Grid& grid) {
        sortClockWise(points);
        auto triangles = triangulate(points);
        auto polygons = partitionConvex(triangles);
        ConcavePolygon concave_poly(polygons);
        for(auto& p : points) {
            p -= concave_poly.getPos();
        }
        auto result = DynamicObject(concave_poly, grid);
        result.model_points = points;
        return result;
    }
    void draw(sf::RenderTarget& render_target) {
        sf::RenderStates states;
        states.transform.translate(transform.getPos());
        states.transform.rotate(transform.getRot() / EPI_PI * 180.f);
        states.transform.translate(-tex_size / 2.f + tex_offset);
        sf::Sprite spr;
        spr.setTexture(tex);
        render_target.draw(spr, states);
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
        int brush_size = 20;
        bool showColliderEdges = true;
    }opt;

    sf::Clock clock_processing_time;
    double processing_time;
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
        physics_manager.steps = 16U;
        event_handler.addCallback(sf::Event::KeyPressed, 
            [&](const sf::Event& e) {
                if(e.key.code == sf::Keyboard::Enter) {
                    auto concave = DynamicObject::create(opt.rigid_construct_points, grid);
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
        event_handler.addCallback(sf::Event::MouseWheelScrolled, 
            [&](const sf::Event& e) {
                if(e.mouseWheelScroll.delta  > 0.f)
                    opt.brush_size += 1;
                if(e.mouseWheelScroll.delta  < 0.f)
                    opt.brush_size -= 1;
            });
        return true;
    }
    void onUpdate() override {
        clock_processing_time.restart();
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
            for(int i = -opt.brush_size / 2; i < opt.brush_size / 2; i++) 
                for(int ii = -opt.brush_size / 2; ii < opt.brush_size / 2; ii++) 
                    grid.set({(int)mouse_pos.x + i , static_cast<int>(grid.height - (int)mouse_pos.y - 1 + ii)}, c);
        };
        drawMaterial(sf::Keyboard::S, eCellType::Sand);
        drawMaterial(sf::Keyboard::Z, eCellType::Stone);
        drawMaterial(sf::Keyboard::X, eCellType::Air);
        static vec2i last_mouse_pos = getMousePos();
        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            vec2f mouse_pos = (vec2f)getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            mouse_pos.y = grid.height - mouse_pos.y - 1;
            float spawn_size = opt.brush_size / 2.f;
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
                for(float y = mouse_pos.y - spawn_size; y < mouse_pos.y + spawn_size; y += ParticleGroup::radius * 2.f) {
                    for(float x = mouse_pos.x - spawn_size; x < mouse_pos.x + spawn_size; x += ParticleGroup::radius * 2.f) {
                        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                            particle_manager.add({x, y}, Cell(eCellType::Water), force);
                        }
                    }
                }
            }
        }
        last_mouse_pos = getMousePos();
        grid.update(Time::deltaTime());
        grid.convertFloatingParticles(particle_manager);
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
        processing_time = clock_processing_time.getElapsedTime().asSeconds();
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
        {
            auto mouse_pos = getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            for(int i = -opt.brush_size / 2; i < opt.brush_size / 2; i++)  {
                for(int ii = -opt.brush_size / 2; ii < opt.brush_size / 2; ii++)  {
                    sf::Vertex vert;
                    vert.position = vec2f(mouse_pos.x + i + 1, static_cast<int>(grid.height - (int)mouse_pos.y - 1 + ii));
                    vert.color = sf::Color::White;
                    vert.color.a = 64;
                    downscaled.draw(&vert, 1U, sf::Points);
                }
            }
        }
        for(auto& object : dynamic_objects) {
            for(const auto& poly : object.collider.getPolygonShape(object.transform)) {
                auto points = poly.getVertecies();

                sf::Vertex vert[points.size()];
                size_t idx = 0;
                for(auto& p : points) {
                    vert[idx].position = p;
                    vert[idx].color = sf::Color::Cyan;
                    idx++;
                }
                downscaled.draw(vert, points.size(), sf::TrianglesFan);
            }
            object.draw(downscaled);
        }

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
        if(opt.showColliderEdges) {
        for(auto& object : dynamic_objects) {
             for(auto poly : object.collider.getPolygonShape(object.transform)) {
                 auto points = poly.getVertecies();
        
                 sf::Vertex vert[2];
                 vert[0].color = vert[1].color = sf::Color::Magenta;
                 vert[0].position = getScreenPos(points.back());
                 for(auto& p : points) {
                     vert[1].position = getScreenPos(p);
                     window.draw(vert, 2U, sf::Lines);
                     vert[0] = vert[1];
                 }
             }
        }
        for(auto& object : grid.terrain_objects.segment_colliders) {
            for(auto poly : object.collider.getPolygonShape(object.transform)) {
                auto points = poly.getVertecies();
                auto points_size = points.size();
        
                sf::Vertex vert[2];
                vert[0].color = vert[1].color = sf::Color::White;
                vert[0].position = getScreenPos(points.back());
                for(auto& p : points) {
                    vert[1].position = getScreenPos(p);
                    window.draw(vert, 2U, sf::Lines);
                    vert[0] = vert[1];
                }
            }
        }
        }

        ImGui::Begin("options");
        auto delta_time = 1.0/double(ImGui::GetIO().Framerate);
        ImGui::Text("Application average %.3f ms/f (%.1f FPS)",
            delta_time * 1000.0, double(ImGui::GetIO().Framerate));

        ImGui::Text("Application processing time usage: %.0f %%", processing_time / delta_time * 100.0);

        ImGui::Text("particle count: %zu", particle_manager.size());
        ImGui::Text("dynamic object count: %zu", dynamic_objects.size());
        ImGui::End();
    }
    Demo(size_t w, size_t h) : App(w, h, "demo"), grid(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE, physics_manager), particle_manager(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE) {}
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
