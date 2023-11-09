#include "app.hpp"
#include "SFML/Window/Event.hpp"
using namespace epi;

void App::run() {
    while (m_window.isOpen()) {
        sf::Event event;
        while (m_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                m_window.close();
            } else {
                m_event_handler.process(event);
            }
        }

        this->onUpdate();
        m_window.clear(sf::Color::Black);
        this->onRender(m_window);
        m_window.display();
    }
}
