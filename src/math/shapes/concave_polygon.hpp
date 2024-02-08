#ifndef EPI_CONCAVE_POLYGON_HPP
#define EPI_CONCAVE_POLYGON_HPP
#include <vector>
#include "convex_polygon.hpp"
#include "math/math_func.hpp"

namespace epi {
struct ConcavePolygon {
private:
    std::vector<ConvexPolygon> m_model;
    std::vector<ConvexPolygon> m_polygons;
    vec2f pos;
    float rotation = 0.f;
    void m_updatePolys() {
        for(int i = 0; i < m_model.size(); i++) {
            m_polygons[i].setRot(m_model[i].getRot() + rotation);
        }
        for(int i = 0; i < m_model.size(); i++) {
            m_polygons[i].setPos(rotateVec(m_model[i].getPos(), rotation) + pos);
        }
    }
public:
    float getRot() const {
        return rotation;
    }
    void setRot(float r) {
        rotation = r;
        m_updatePolys();
    }
    vec2f getPos() const {
        return pos;
    }
    void setPos(vec2f v) {
        pos = v;
        m_updatePolys();
    }
    const std::vector<ConvexPolygon>& getModelPolygons() const {
        return m_model;
    }
    const std::vector<ConvexPolygon>& getPolygons() const {
        return m_polygons;
    }
    ConcavePolygon(std::vector<ConvexPolygon> polygons);

};
}
#endif
