#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "grid.hpp"
#include "imgui.h"
#include "particles.hpp"
#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "timer.h"
#include "templates/component_group.hpp"
#include "templates/primitive_wrapper.hpp"
#include "utils/time.hpp"


using namespace epi;

#include <gtest/gtest.h>

#define GRID_DOWNSCALE 2
class Demo : public App {
public:
    Grid grid;
    ParticleManager particle_manager;
    sf::RenderTexture downscaled;
    ThreadPool tp;
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
        return true;
    }
    void onUpdate() override {
        int brush_size = 10;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S) || sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
            auto mouse_pos = getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            Cell c(eCellType::Air);
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                c= Cell(eCellType::Sand);
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                c= Cell(eCellType::Water);
            for(int i = 0; i < brush_size; i++) {
                grid.set({(int)mouse_pos.x + i , static_cast<int>(grid.height - (int)mouse_pos.y - 1)}, c);
            }
        }
        static vec2i last_mouse_pos = getMousePos();
        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            vec2f mouse_pos = (vec2f)getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            mouse_pos.y = grid.height - mouse_pos.y - 1;
            const int spawn_size = 10;
            static size_t tick_count = 0;
            tick_count++;
            auto force = vec2f(getMousePos() - last_mouse_pos) * 1000.f;
            for(float y = mouse_pos.y; y < mouse_pos.y + spawn_size; y += Particle::radius) {
                for(float x = mouse_pos.x; x < mouse_pos.x + spawn_size; x += Particle::radius) {
                    auto& box = *particle_manager.boxOf({x, y});
                    auto adj = particle_manager.getAdjacentBoxes(box);
                    bool canSpawn = true;
                    if(box.size() != 0)
                        canSpawn = false;
                    for(int i = 0; i < 8; i++) {
                        if(adj[i]->size() != 0) {
                            canSpawn = false;

                            break;
                        }
                    }
                    if(canSpawn) {
                        auto p = new Particle{vec2f(x, y ), Cell(eCellType::Water)};
                        p->acc += vec2f(force.x, -force.y);
                        particle_manager.add(std::move(std::unique_ptr<Particle>{p}));
                        box.push_back(p);
                    }
                }
            }
        }
        last_mouse_pos = getMousePos();
        grid.update(Time::deltaTime());
        particle_manager.update(Time::deltaTime(), grid, &tp);
    }
    void onRender(sf::RenderWindow& window) override {
        grid.render(downscaled);
        particle_manager.render(downscaled);

        sf::Sprite spr;
        spr.setPosition(0, 0);
        spr.setTexture(downscaled.getTexture());
        float scalar = pixelSize(window.getSize());
        spr.setScale(scalar, scalar);
        window.draw(spr);

        ImGui::Begin("options");
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
        1000.0/double(ImGui::GetIO().Framerate), double(ImGui::GetIO().Framerate));
        ImGui::Text("particle count: %zu", particle_manager.particles.size());
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
