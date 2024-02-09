#include "transform.hpp"
namespace epi {
std::vector<vec2f> Transform::transformPoints(std::vector<vec2f> points, const sf::Transform& transform) {
    for(auto& p : points) {
        p = transform.transformPoint(p);
    }
    return points;
}

}
