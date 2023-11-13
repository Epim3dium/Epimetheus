#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "io/app.hpp"
#include "scene/debug_shape.hpp"
#include "scene/entity.hpp"
#include "templates/group.hpp"
#include "scene/component_group.hpp"
#include "timer.h"
#include "scene/transform.hpp"


using namespace epi;

#include <gtest/gtest.h>

class Demo : public App {
    public:
    Entities entities;
    Transforms transforms;
    DebugShapes debug_shapes;
    void onUpdate() override {
        entities.update();
        transforms.update();
    }
    void onRender(sf::RenderWindow& window) override {
        //debug_shapes.render(window);
    }
};

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
    std::cerr << "ERROR";
}
