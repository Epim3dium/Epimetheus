#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "grid.hpp"
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
        for(int x = 0; x < grid.width; x++) {
            for(int y = 0; y < grid.height; y++) {
                if(x == 0 || y == 0 || x == grid.width - 1 || y == grid.height - 1) {
                    grid.at(x, y) = Cell(eCellType::Bedrock);
                }
            }
        }
        return true;
    }
    void onUpdate() override {
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
            auto mouse_pos = getMousePos();
            auto px_size = grid.pixelSize({static_cast<unsigned int>(width), static_cast<unsigned int>(height)});
            mouse_pos.x /= px_size;
            mouse_pos.y /= px_size;
            grid.at((int)mouse_pos.x , grid.height - (int)mouse_pos.y - 1) = Cell(eCellType::Sand);
        }
        grid.update();
    }
    void onRender(sf::RenderWindow& window) override {
        grid.render(window);
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
    Demo d(800, 800);
    d.run();
}
