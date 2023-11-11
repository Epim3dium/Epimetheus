#include "app.hpp"
#include "SFML/Window/Event.hpp"
#include "utils/time.hpp"
using namespace epi;

void App::run() {
    if(!onSetup())
        return;
    m_window.setFramerateLimit(60U);
    while (m_window.isOpen()) {
        sf::Event event;
        while (m_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                m_window.close();
            } else {
                event_handler.process(event);
            }
        }
        Time::update();

        this->onUpdate();
        m_window.clear(sf::Color::Black);
        this->onRender(m_window);
        m_window.display();
    }
}
