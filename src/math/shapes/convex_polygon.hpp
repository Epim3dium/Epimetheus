#ifndef EPI_CONVEX_POLYGON_HPP
#define EPI_CONVEX_POLYGON_HPP
#include "math/math_defs.hpp"
#include <vector>
#include "AABB.hpp"
namespace epi {
class ConvexPolygon {
    std::vector<vec2f> points;
    std::vector<vec2f> model;
    float rotation;
    vec2f pos;
    vec2f scale = {1, 1};
    void m_updatePoints() {
        for(size_t i = 0; i < model.size(); i++) {
            const auto& t = model[i];
            points[i].x = (t.x * cosf(rotation) - t.y * sinf(rotation)) * scale.x;
            points[i].y = (t.x * sinf(rotation) + t.y * cosf(rotation)) * scale.y;
            points[i] += pos;
        }
    }
    static std::vector<vec2f> m_avgPoints(std::vector<vec2f> model) {
        vec2f avg = vec2f(0, 0);
        for(auto& p : model) {
            avg += p;
        }
        avg /= (float)model.size();
        for(auto& p : model) {
            p -= avg;
        }
        return model;
    }
public:
    float getRot() const {
        return rotation;
    }
    void setRot(float r) {
        rotation = r;
        m_updatePoints();
    }
    vec2f getPos() const {
        return pos;
    }
    void setPos(vec2f v) {
        pos = v;
        m_updatePoints();
    }
    void setScale(vec2f s) {
        scale = s;
        m_updatePoints();
    }
    vec2f getScale() const {
        return scale;
    }
    const std::vector<vec2f>& getVertecies() const {
        return points;
    }
    const std::vector<vec2f>& getModelVertecies() const {
        return model;
    }
    ConvexPolygon() {}
    ConvexPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : points(model_.size(), vec2f(0, 0)), model(model_), rotation(rot_), pos(pos_) {
        std::sort(model.begin(), model.end(), [](vec2f a, vec2f b) {
            return atan2(a.y, a.x) < atan2(b.y, b.x);
        });
        m_updatePoints();
        //m_avgPoints(model);
    }

    static ConvexPolygon CreateRegular(vec2f pos, float rot, size_t count, float dist);
    static ConvexPolygon CreateFromAABB(const AABB& aabb);
    static ConvexPolygon CreateFromPoints(std::vector<vec2f> verticies);

    friend void draw(sf::RenderWindow& rw, const ConvexPolygon& poly, sf::Color clr);
    friend void drawFill(sf::RenderTarget& rw, const ConvexPolygon& poly, sf::Color clr);
    friend void drawOutline(sf::RenderTarget& rw, const ConvexPolygon& poly, sf::Color clr);
};

}
#endif
