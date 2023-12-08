#include "types.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Vector3.hpp"
#include "math/geometry_func.hpp"
#include "physics/col_utils.hpp"

#include <cmath>
#include <math.h>
#include <numeric>
#include <vector>
namespace epi {
std::vector<ConvexPolygon> toTriangles(const std::vector<vec2f>& points) {
    std::vector<ConvexPolygon> result;
    std::vector<vec2f> tmp = points;
    float angle_counter = 0.f;
    for(int i = 0; i < tmp.size(); i++) {
        auto first = tmp[i];
        auto mid = tmp[(i + 1) % tmp.size()];
        auto last = tmp[(i + 2) % tmp.size()];
        angle_counter += angleAround(first, mid, last);
    }
    if(angle_counter < 0.f) {
        std::reverse(tmp.begin(), tmp.end());
    }
    while(result.size() != points.size() - 2) {
        size_t res_size_last = result.size();
        for(int i = 0; i < tmp.size(); i++) {
            auto first = tmp[i];
            auto mid_index = (i + 1) % tmp.size();
            auto mid = tmp[mid_index];
            auto last = tmp[(i + 2) % tmp.size()];
            float angle = angleAround(first, mid, last);
            if(angle < 0.f) {
                continue;
            }
            bool containsVertex = false;
            for(auto p : tmp) {
                if(p == first || p == mid || p == last)
                    continue;
                if(isOverlappingPointPoly(p, {first, mid, last})) {
                    containsVertex = true;
                    break;
                }
            }
            if(containsVertex) {
                continue;
            }
            result.push_back(ConvexPolygon::CreateFromPoints({first, mid, last}));
            tmp.erase(tmp.begin() + mid_index);
            break;
        }
        if(res_size_last == result.size()) {
            break;
        }
    }
    return result;
}
bool hasSharedEdge(std::vector<vec2f> points0, const std::vector<vec2f>& points1) {
    std::reverse(points0.begin(), points0.end());

    auto last0 = points0.back();
    for(auto vert0 : points0) {
        auto last1 = points1.back();
        for(auto vert1 : points1) {
            if(vert0 == vert1 && last0 == last1)
                return true;
            last1 = vert1;
        }
        last0 = vert0;
    }
    return false;
}
std::pair<bool, std::vector<vec2f>> tryMergingToConvex(vec2f avg, const std::vector<vec2f>& points0, const std::vector<vec2f>& points1) {
    std::vector<vec2f> point_merge = points0;
    point_merge.insert(point_merge.end(), points1.begin(), points1.end());
    for(auto& p : point_merge)
        p -= avg;
    std::sort(point_merge.begin(), point_merge.end(), [](vec2f a, vec2f b) {
        return atan2(a.y, a.x) > atan2(b.y, b.x);
    });
    for(auto& p : point_merge)
        p += avg;
    for(int i = 0; i < point_merge.size() - 1; i++) {
        if(point_merge[i] == point_merge[i + 1]) {
            point_merge.erase(point_merge.begin() + i);
            i--;
        }
    }

    for(int i = 0; i < point_merge.size(); i++) {
        auto first = point_merge[i];
        auto mid = point_merge[(i + 1) % point_merge.size()];
        auto last = point_merge[(i + 2) % point_merge.size()];
        if(angleAround(first, mid, last) < 0.f)
            return {false, {}};
    }
    return {true, point_merge};
}
std::vector<ConvexPolygon> toBiggestConvexPolygons(const std::vector<ConvexPolygon>& polygons) {
    std::vector<bool> wasIncluded(polygons.size(), false);
    std::vector<ConvexPolygon> result;
    for(int i = 0; i < polygons.size(); i++) {
        if(wasIncluded[i])
            continue;
        ConvexPolygon current = polygons[i];
        wasIncluded[i] = true;
        bool wasFound = true;
        while(wasFound) {
            wasFound = false;
            for(int ii = i + 1; ii < polygons.size(); ii++) {
                if(wasIncluded[ii])
                    continue;
                if(!hasSharedEdge(current.getVertecies(), polygons[ii].getVertecies())) 
                    continue;
                auto convexMerge = tryMergingToConvex(current.getPos(), current.getVertecies(), polygons[ii].getVertecies());
                if(!convexMerge.first)
                    continue;
                current = ConvexPolygon::CreateFromPoints(convexMerge.second);
                wasIncluded[ii] = true;
                wasFound = true;
                break;
            }
        }
        result.push_back(current);
    }
    return result;
}
AABB AABB::CreateFromCircle(const Circle& c) {
    return AABB::CreateMinMax(c.pos - vec2f(c.radius, c.radius), c.pos + vec2f(c.radius, c.radius));
}
AABB AABB::CreateFromVerticies(const std::vector<vec2f>& verticies) {
    vec2f min = {INFINITY, INFINITY};
    vec2f max = {-INFINITY, -INFINITY};
    for(auto& v : verticies) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
    }
    return AABB::CreateMinMax(min, max);
}
AABB AABB::CreateFromPolygon(const ConvexPolygon& p) {
    return AABB::CreateFromVerticies(p.getVertecies());
}
AABB AABB::CreateMinMax(vec2f min, vec2f max) {
    AABB a;
    a.min = min;
    a.max = max;
    return a;
}
AABB AABB::CreateCenterSize(vec2f center, vec2f size) {
    AABB a;
    a.setCenter(center);
    a.setSize(size);
    return a;
}
AABB AABB::CreateMinSize(vec2f min, vec2f size) {
    AABB a;
    a.min = min;
    a.max = a.min + size;
    return a;
}
Ray Ray::CreatePoints(vec2f a, vec2f b) {
    Ray r;
    r.pos = a;
    r.dir = b - a;
    return r;
}
Ray Ray::CreatePositionDirection(vec2f p, vec2f d) {
    Ray r;
    r.pos = p;
    r.dir = d;
    return r;
}
void draw(sf::RenderWindow& rw, const ConvexPolygon& poly, Color clr) {
    struct VertPair {
        sf::Vertex a;
        sf::Vertex b;
    };
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[2];
        t[0].color = clr;
        t[1].color = clr;
        t[0].position = poly.getVertecies()[i];
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 2, sf::Lines);
    }
}
void drawOutline(sf::RenderTarget& rw, const AABB& aabb, Color clr) {
    sf::Vertex t[5];
    vec2f vert[] = {aabb.bl(), aabb.br(), aabb.tr(), aabb.tl(), aabb.bl()};
    for(int i = 0; i < 5; i++) {
        t[i].color = clr;
        t[i].position = vert[i];
    }
    rw.draw(&t[0], 2, sf::Lines);
    rw.draw(&t[1], 2, sf::Lines);
    rw.draw(&t[2], 2, sf::Lines);
    rw.draw(&t[3], 2, sf::Lines);
}
void drawFill(sf::RenderTarget& rw, const AABB& aabb, Color clr) {
    sf::Vertex t[4];
    vec2f vert[] = {aabb.bl(), aabb.br(), aabb.tr(), aabb.tl()};
    for(int i = 0; i < 4; i++) {
        t[i].color = clr;
        t[i].position = vert[i];
    }
    rw.draw(t, 4, sf::Quads);
}
void drawFill(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr) {
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[3];
        t[0].color = clr;
        t[1].color = clr;
        t[2].color = clr;
        t[0].position = poly.getVertecies()[i];
        t[2].position = poly.getPos();
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 3, sf::Triangles);
    }
}
void drawOutline(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr) {
    for(size_t i = 0; i < poly.getVertecies().size(); i++) {
        sf::Vertex t[2];
        t[0].color = clr;
        t[1].color = clr;
        t[0].position = poly.getVertecies()[i];
        if(i != poly.getVertecies().size() - 1) {
            t[1].position = poly.getVertecies()[i + 1];
        } else {
            t[1].position = poly.getVertecies()[0];
        }
        rw.draw(t, 2, sf::Lines);
    }
}
ConvexPolygon ConvexPolygon::CreateRegular(vec2f pos, float rot, size_t count, float dist) {
    std::vector<vec2f> model;
    for(size_t i = 0; i < count; i++) {
        model.push_back(vec2f(sinf(3.141f * 2.f * ((float)i / (float)count)), cosf(3.141f * 2.f * ((float)i / (float)count))) * dist );
    }
    return ConvexPolygon(pos, rot, model);
}
ConvexPolygon ConvexPolygon::CreateFromPoints(std::vector<vec2f> verticies) {
    vec2f avg = std::reduce(verticies.begin(), verticies.end()) / (float)verticies.size();
    for(auto& v : verticies)
        v -= avg;
    return ConvexPolygon(avg, 0.f, verticies);
}
ConvexPolygon ConvexPolygon::CreateFromAABB(const AABB& aabb) {
    std::vector<vec2f> points = {aabb.min, vec2f(aabb.min.x, aabb.max.y), aabb.max, vec2f(aabb.max.x, aabb.min.y)};
    return ConvexPolygon::CreateFromPoints(points);
}


vec2f operator* (vec2f a, vec2f b) {
    return vec2f(a.x * b.x, a.y * b.y);
}

}
