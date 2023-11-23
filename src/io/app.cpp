#include "app.hpp"
#include "SFML/Window/Event.hpp"
#include "utils/time.hpp"
#include "imgui-SFML.h"
#include "imgui.h"
using namespace epi;

void App::run() {
    if(!onSetup())
        return;
    if (!ImGui::SFML::Init(m_window)) return;
    m_window.setFramerateLimit(60U);
    sf::Clock sfml_clock;
    while (m_window.isOpen()) {
        sf::Event event;
        while (m_window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(m_window, event);
            if (event.type == sf::Event::Closed) {
                m_window.close();
            } else if (event.type == sf::Event::Resized) {
                sf::View view;
                // resize my view
                view.setSize({
                        static_cast<float>(event.size.width),
                        static_cast<float>(event.size.height)
                });
                view.setCenter(event.size.width / 2.f, event.size.height / 2.f);
                m_window.setView(view);
            } else {
                event_handler.process(event);
            }
        }
        ImGui::SFML::Update(m_window, sfml_clock.restart());
        Time::update();

        this->onUpdate();
        m_window.clear(sf::Color::Black);
        this->onRender(m_window);
        ImGui::SFML::Render(m_window);
        m_window.display();
    }
}
