#ifndef EPI_INPUT_HANDLER_HPP
#define EPI_INPUT_HANDLER_HPP
#include "SFML/Window/Event.hpp"
#include "debug/log.hpp"
#include <map>
namespace epi {

class EventHandler {
    std::multimap<sf::Event::EventType, std::function<void(const sf::Event&)>> m_callbacks;
public:
    void addCallback(sf::Event::EventType type, std::function<void(const sf::Event&)> callback) {
        m_callbacks.insert({type, callback});
    }
    void process(const sf::Event& e) {
        auto [begin, end] = m_callbacks.equal_range(e.type);
        for(auto itr = begin; itr != end; itr++) {
            itr->second(e);
        }
    }
};

}
#endif
