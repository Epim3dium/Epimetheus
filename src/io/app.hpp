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
public:
    inline auto getMousePos() const {
        return sf::Mouse::getPosition(m_window);
    }
    inline auto getSize() const {
        return m_window.getSize();
    }
    virtual ~App() {std::cerr << "app destroyed\n";}
    const size_t width;
    const size_t height;
    const std::string title;

    virtual bool onSetup() { return true; }
    virtual void onUpdate() = 0;
    virtual void onRender(sf::RenderWindow& window) = 0;
    void run();
    App(unsigned int width, unsigned int height, std::string title)
        : width(width), height(height), m_window(sf::VideoMode{width, height}, title), title(title) {}
};

};     // namespace epi
#endif // EPI_APP_HPP
