#pragma once
#include "grid.hpp"
#include "imgui.h"
#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "physics/physics_manager.hpp"
#include "timer.h"
#include "utils/time.hpp"
namespace epi {
struct DynamicObject {
    //rigidbody stuff
    Transform transform;
    Collider collider;
    Rigidbody rigidbody;
    Material material;

    //texture stuff
    sf::Texture tex;
    vec2f tex_size;
    vec2f tex_offset;

    std::vector<vec2i> personal_barriers;

    //collision with particles
    std::vector<vec2f> model_points;
    std::vector<vec2f> getVerticies() const {
        std::vector<vec2f> points(model_points.size());
        const auto rotation = transform.getRot();
        const auto pos = transform.getPos();
        for(size_t i = 0; i < model_points.size(); i++) {
            const auto& t = model_points[i];
            points[i].x = (t.x * cosf(rotation) - t.y * sinf(rotation));
            points[i].y = (t.x * sinf(rotation) + t.y * cosf(rotation));
            points[i] += pos;
        }
        return points;
    }


    RigidManifold getManifold() {
        RigidManifold man;
        man.transform =&transform;
        man.rigidbody =&rigidbody;
        man.material = &material;
        man.collider = &collider;
        return man;
    }
    void m_carveShape(ConcavePolygon shape, Grid& grid) {
        AABB bounding_box = AABB::CreateFromPolygon(shape.getPolygons().front());
        for(auto& p : shape.getPolygons())
            bounding_box = bounding_box.combine(AABB::CreateFromPolygon(p));
        sf::Image rend_tex;

        rend_tex.create(bounding_box.size().x, bounding_box.size().y, sf::Color::Transparent);
        tex_size = bounding_box.size();
        tex_offset = -shape.getPos() + bounding_box.center();

        for(int y = bounding_box.bottom(); y < bounding_box.top(); y++) {
            for(int x = bounding_box.left(); x < bounding_box.right(); x++) {
                bool isInside = false;
                for(auto convex : shape.getPolygons()) {
                    if(isOverlappingPointPoly(vec2f(x, y) + vec2f(0.5f, 0.5f), convex.getVertecies())) {
                        isInside = true;
                    }
                }
                if(!isInside) {
                    continue;
                }
                sf::Vertex vert;
                auto cell = grid.get(x, y);
                if(cell.type == eCellType::Air)
                    continue;
                auto coord = vec2f(x, y) - bounding_box.min;
                rend_tex.setPixel(coord.x, coord.y, cell.color);
                grid.set({x, y}, Cell(eCellType::Air));
            }
        }
        tex.loadFromImage(rend_tex);

    }
    DynamicObject(ConcavePolygon shape, Grid& grid) : collider(shape) {
        m_carveShape(shape, grid);
        transform.setPos(shape.getPos());
    }
    static void sortClockWise(std::vector<vec2f>& points) {
        int sign = 0;
        for(int i = 0; i < points.size(); i++) {
            auto first = points[i];
            auto mid = points[(i + 1) % points.size()];
            auto last = points[(i + 2) % points.size()];
            float angle = angleAround(first, mid, last);
            sign += angle > 0.f ? 1 : -1;
        }
        if(sign < 0) {
            std::reverse(points.begin(), points.end());
        }
    }
    static DynamicObject create(std::vector<vec2f> points, Grid& grid) {
        sortClockWise(points);
        auto triangles = triangulate(points);
        auto polygons = partitionConvex(triangles);
        ConcavePolygon concave_poly(polygons);
        for(auto& p : points) {
            p -= concave_poly.getPos();
        }
        auto result = DynamicObject(concave_poly, grid);
        result.model_points = points;
        return result;
    }
    void draw(sf::RenderTarget& render_target) {
        sf::RenderStates states;
        states.transform.translate(transform.getPos());
        states.transform.rotate(transform.getRot() / EPI_PI * 180.f);
        states.transform.translate(-tex_size / 2.f + tex_offset);
        sf::Sprite spr;
        spr.setTexture(tex);
        render_target.draw(spr, states);
    }
};

}
