#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "scene/debug_shape.hpp"
#include "scene/entity.hpp"
#include "scene/script.hpp"
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
    Scripts scripts;
    bool onSetup() override {
        static bool isFirst = true;
        event_handler.addCallback(sf::Event::EventType::MouseButtonPressed, 
            [&](const sf::Event& e) {
                auto id = entities.create();
                sf::Vector2f pos = {static_cast<float>(e.mouseButton.x), static_cast<float>(e.mouseButton.y)};
                transforms.create(id, pos, 0.f, {1.f, 1.f});
                debug_shapes.createCircle(id, 5.f, sf::Color::Red);
                if(isFirst) {
                    isFirst = false;
                    scripts.create(id, 
                        [&](Entities::ID_t id) {
                            auto pos = transforms.getByID<sf::Vector2f>(eTransform::position_vec2f, id);
                            if(pos.has_value()) {
                                pos->get().x += sf::Keyboard::isKeyPressed(sf::Keyboard::D) * 10.f;
                                pos->get().x -= sf::Keyboard::isKeyPressed(sf::Keyboard::A) * 10.f;
                                pos->get().y += sf::Keyboard::isKeyPressed(sf::Keyboard::S) * 10.f;
                                pos->get().y -= sf::Keyboard::isKeyPressed(sf::Keyboard::W) * 10.f;
                            }
                        });
                }
            });
        event_handler.addCallback(sf::Event::EventType::KeyPressed, 
            [&](const sf::Event& e) {
            if(e.key.code == sf::Keyboard::P) {
                entities.debugPrintHierarchy();
            }
            });
        return true;
    }
    void onUpdate() override {
        entities.update();
        transforms.update();
        scripts.onUpdate();
    }
    void onRender(sf::RenderWindow& window) override {
        debug_shapes.render(window, transforms);
        scripts.onRender(window);
    }
    Demo(unsigned w, unsigned h) : App(w, h, "demo") {}
};

int main(int argc, char** argv)
{
    Log::ReportingLevel = LogLevel::WARNING;
    testing::InitGoogleTest(&argc, argv);
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
    Log::ReportingLevel = LogLevel::DEBUG3;
    Demo demo(1000, 500);
    demo.run();
}
