#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>
#include <unordered_set>

#include "group.hpp"
#include "primitive_wrapper.hpp"
using namespace epi;

struct Mass : PrimitiveWrapper<float> {
};
struct Friction : PrimitiveWrapper<float> {
};
int main()
{
    Friction fric;
    
    Group<Friction, Mass> g;
    Entity e;
    g.push_back(e, {2.1}, {69.f});
    g.push_back(Entity(), {6.9}, {2.f});
    g.push_back(Entity(), {3.7}, {3.f});
    g.push_back(Entity(), {4.2}, {4.f});
    auto clip = g.createClipping<Friction, Mass>();
    
    std::get<0U>(*clip.begin());
    for(auto [f, m] : clip) {
        std::cout << f << " " << m << "\n";
    }
    // create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "My window");

    // run the program as long as the window is open
    while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // clear the window with black color
        window.clear(sf::Color::Black);

        // draw everything here...
        // window.draw(...);

        // end the current frame
        window.display();
    }

    return 0;
}
