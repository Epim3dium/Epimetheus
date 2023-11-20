#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "grid.hpp"
#include "imgui.h"
#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "timer.h"
#include "templates/component_group.hpp"
#include "templates/primitive_wrapper.hpp"


using namespace epi;

#include <gtest/gtest.h>

class Demo : public App {
public:
    Grid grid;
    bool onSetup() override { 
        return true;
    }
    void onUpdate() override {
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
            auto mouse_pos = getMousePos();
            auto px_size = grid.pixelSize(getSize());
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            for(int i = 0; i < 10; i++) {
                auto c= Cell(eCellType::Sand);
                grid.set({(int)mouse_pos.x + i , static_cast<int>(grid.height - (int)mouse_pos.y - 1)}, c);
            }
        }
        grid.update();
    }
    void onRender(sf::RenderWindow& window) override {
        grid.render(window);
        ImGui::Begin("options");
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
        1000.0/double(ImGui::GetIO().Framerate), double(ImGui::GetIO().Framerate));
        ImGui::End();
    }
    Demo(size_t w, size_t h) : App(w, h, "demo"), grid(w / 4, h / 4) {}
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
}
