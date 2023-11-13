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
    result.setFillColor(color);

    m_group->push_back(parent, result);
}
void DebugShapes::erase(Entities::ID_t id) {
    m_group->eraseByID(id);
}
