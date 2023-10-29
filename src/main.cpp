#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "system.hpp"
#include "timer.h"


using namespace epi;

void f(float x, float xx) {
    std::cout << x << "\t" << xx << "\n";
}
enum class eVelocity {
    velx,
    vely,
    velz,
};
int main()
{
    System<eVelocity>::Factory fac;

    fac.add<float>(eVelocity::velx);
    fac.add<float>(eVelocity::vely);

    auto sys = fac.create();
    sys->push_back(21.f, 37.f);
    sys->push_back(6.f, 9.f);

    sys->update(
            {eVelocity::velx, eVelocity::vely}, 
            [](float x, float xx) {
    std::cout << x << "\t" << xx << "\n";
    });
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
