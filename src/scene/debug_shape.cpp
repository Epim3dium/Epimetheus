#include "debug_shape.hpp"
#include "SFML/Graphics/ConvexShape.hpp"
#include "math/math.hpp"
#include <numeric>

using namespace epi;

void DebugShapes::render(sf::RenderTarget& window, const Transforms& trans) {
    sf::RenderStates states;

    updateMatching<eDebugShape, eTransform>(*m_group.get(), *trans.m_group.get(), {eDebugShape::shape_sf}, {eTransform::transform_sf}, 
        [&window](const sf::ConvexShape& shape, const sf::Transform& trans) {
            window.draw(shape, {trans});
        });
}
void DebugShapes::create(Entities::ID_t parent, std::vector<sf::Vector2f> points,
            sf::Color color) 
{
    sf::ConvexShape result;

    sort_clockwise(points.begin(), points.end());
    for(auto a = points.begin(), b = std::next(a), c = std::next(b); c != points.end(); a++, b++, c++) {
        if(angle(*b, *a, *c) > EPI_PI) {
            throw std::logic_error("cannot draw concave shapes");
        }
    }

    result.setPointCount(points.size());
    for(int i = 0; i < points.size(); i++) {
        result.setPoint(i, points[i]);
    }
    result.setOutlineThickness(1.f);
    result.setOutlineColor(color);
    result.setFillColor(sf::Color::Transparent);

    m_group->push_back(parent, result);
}
void DebugShapes::createCircle(Entities::ID_t parent, float radius,
            sf::Color color, size_t vert_count) 
{
    std::vector<sf::Vector2f> points;
    for(float x = 0.f; x < EPI_PI * 2.f; x += EPI_PI * 2.f / vert_count) {
        points.push_back(rotate(sf::Vector2f(radius, 0.f), x));
    }
    create(parent, points, color);
}
void DebugShapes::erase(Entities::ID_t id) {
    m_group->eraseByID(id);
}
