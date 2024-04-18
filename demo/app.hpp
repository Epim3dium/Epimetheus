#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>
#include <unordered_set>

#include "core/hierarchy.hpp"
#include "core/group.hpp"
#include "debug/log.hpp"
#include "memory/linear_allocator.hpp"
#include "physics/collider.hpp"
#include "physics/physics_manager.hpp"
#include "physics/rigidbody.hpp"
#include "templates/primitive_wrapper.hpp"
#include "imgui-SFML.h"
#include "imgui.h"
#include "imgui_internal.h"

namespace epi {
class Application {
private:
    sf::RenderWindow m_window;
    std::multimap<sf::Event::EventType, std::function<void(sf::Event)> > m_event_hooks;
    int m_constant_framerate_set = 0;
protected:
    virtual bool setup() {
        return true;
    }
    virtual void update(sf::Time deltaTime) {
    }
    virtual void render(sf::RenderTarget& window) {
    }
    virtual void cleanup() {
    }
public:
    void addHook(sf::Event::EventType type, std::function<void(sf::Event)> callback) {
        m_event_hooks.insert({type, callback});
    }
    void setConstantFramerate(int framerate) {
        m_window.setFramerateLimit(framerate);
        m_constant_framerate_set = framerate;
    }
    int run() {
        if (!ImGui::SFML::Init(m_window)) {
            return 1;
        }
        if(!setup()) {
            return 2;
        }
        sf::Clock delTclock;
        while (m_window.isOpen()) {
            // check all the window's events that were triggered since the last
            // iteration of the loop
            sf::Event event{};
            auto delTtime = delTclock.restart();
            if(m_constant_framerate_set != 0) {
                double seconds = 1.0 / static_cast<double>(m_constant_framerate_set);
                delTtime = sf::seconds(seconds);
            }
            
            while (m_window.pollEvent(event)) {
                ImGui::SFML::ProcessEvent(m_window, event);
                if (event.type == sf::Event::Closed)
                    m_window.close();
                
                auto begin = m_event_hooks.lower_bound(event.type);
                auto end = m_event_hooks.upper_bound(event.type);
                
                for(auto itr = begin; itr != end; itr++) {
                    itr->second(event);
                }
            }
            ImGui::SFML::Update(m_window, delTtime);
            update(delTtime);
            
            m_window.clear();
            render(m_window);
            ImGui::SFML::Render(m_window);
            m_window.display();
        }
        return 0;
    }
    Application(std::string title, sf::Vector2i size)
        : m_window(sf::VideoMode(size.x, size.y), title, sf::Style::Close) {}
};
}
