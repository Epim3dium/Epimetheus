#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "core/group.hpp"
#include "timer.h"


using namespace epi;

void f(float x, float xx, sf::Vector2f v, bool b) {
    std::cout << x << "\t" << xx << "\t" << v.x << "\t" << b << "\n";
}
enum class eVelocity {
    velx,
    vely,
    velz,
};
enum class eCollider {
    ColNormal,
    isStatic,
};
int main()
{
    Group<eVelocity>::Factory fac1;

    fac1.add<float>(eVelocity::velx);
    fac1.add<float>(eVelocity::vely);
    auto group1 = fac1.create();

    Group<eCollider>::Factory fac2;

    fac2.add<sf::Vector2f>(eCollider::ColNormal);
    fac2.add<bool>(eCollider::isStatic);
    auto group2 = fac2.create();

    group2->push_back(sf::Vector2f(), false);
    group2->push_back(sf::Vector2f(), true);
    group1->push_back(21.f, 37.f);
    group1->push_back(6.f, 9.f);

    group1->update(
        {eVelocity::velx, eVelocity::vely}, 
        [](float& x, float xx) {
            std::cout << x++ << "\t" << xx << "\n";
        }
    );
        updateMatching<eVelocity, eCollider>(f,
            Group<eVelocity>::VariableGetter{group1.get(), {eVelocity::velx, eVelocity::vely}},
            Group<eCollider>::VariableGetter{group2.get(), {eCollider::ColNormal, eCollider::isStatic}}
            );
    return 0;
}
