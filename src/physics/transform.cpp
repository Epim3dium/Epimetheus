#include "transform.hpp"
namespace epi {
std::vector<vec2f> transformPoints(std::vector<vec2f> points, const Transform& transform) {
    for(auto& p : points) {
        p = transform.transformPoint(p);
    }
    return points;
}

}
