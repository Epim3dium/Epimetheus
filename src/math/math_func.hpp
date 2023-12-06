#ifndef EPI_MATH_FUNC_HPP
#define EPI_MATH_FUNC_HPP
#include "SFML/System/Vector2.hpp"
#include <vector>
#include "math_defs.hpp"
namespace epi {

void sort_clockwise(std::vector<sf::Vector2f>::iterator begin, std::vector<sf::Vector2f>::iterator end);
sf::Vector2f rotate(sf::Vector2f vec, float angle);
float length(sf::Vector2f v);
float qlen(vec2f);
vec2f normal(vec2f v);
float dot(sf::Vector2f a, sf::Vector2f b);
vec2f proj(vec2f a, vec2f plane_norm);
float cross(vec2f a, vec2f b);

float angleAround(vec2f a, vec2f pivot, vec2f b);
//around origin point
float angle(vec2f, vec2f);
vec2f sign(vec2f);
vec2f rotateVec(vec2f vec, float angle);


}
#endif// EPI_MATH_FUNC_HPP
