#ifndef EPI_APP_HPP
#define EPI_APP_HPP
#include "SFML/Graphics/RenderWindow.hpp"
#include "event_handler.hpp"
#include <string>
namespace epi {

class App {
    sf::RenderWindow m_window;

protected:
    EventHandler event_handler;
    float delT = 1.f / 60.f;
public:
    const std::string title;

    virtual bool onSetup() { return true; }
    virtual void onUpdate() = 0;
    virtual void onRender(sf::RenderWindow& window) = 0;
    void run();
    App(unsigned int width, unsigned int height, std::string title)
        : m_window(sf::VideoMode{width, height}, title), title(title) {}
};

};     // namespace epi
#endif // EPI_APP_HPP
