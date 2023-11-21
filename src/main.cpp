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
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S) || sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
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
        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            auto mouse_pos = getMousePos();
            auto px_size = pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            mouse_pos.y = grid.height - mouse_pos.y - 1;
            bool canSpawn = true;
            for(auto dy : {0, -1, 1})
                for(auto dx : {0, -1, 1})
                    for(auto p : *particle_manager.boxOF((vec2f)mouse_pos + vec2f(dx, dy))) {
                        if(length(p->pos - (vec2f)mouse_pos) < Particle::radius * 2.f)
                            canSpawn = false;
                    }
            if(canSpawn)
                particle_manager.add(std::unique_ptr<Particle>{new Particle{vec2f(mouse_pos.x, mouse_pos.y ), Cell(eCellType::Sand)}});
        }
        grid.update(epi::Time::deltaTime());
        particle_manager.update(Time::deltaTime(), grid);
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
        ImGui::End();
    }
    Demo(size_t w, size_t h) : App(w, h, "demo"), grid(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE), particle_manager(w >> GRID_DOWNSCALE, h >> GRID_DOWNSCALE) {}
};


int main(int argc, char** argv)
{
    Log::ReportingLevel = LogLevel::WARNING;
    testing::InitGoogleTest(&argc, argv);
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
    Demo d(1024, 1024);
    d.run();
    std::cerr << "stopped";
}
