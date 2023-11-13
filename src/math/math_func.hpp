#ifndef EPI_MATH_FUNC_HPP
#define EPI_MATH_FUNC_HPP
#include "SFML/System/Vector2.hpp"
#include <vector>
namespace epi {

void sort_clockwise(std::vector<sf::Vector2f>::iterator begin, std::vector<sf::Vector2f>::iterator end);
float angle(sf::Vector2f pivot, sf::Vector2f a, sf::Vector2f b);
sf::Vector2f rotate(sf::Vector2f vec, float angle);

}
#endif// EPI_MATH_FUNC_HPP
