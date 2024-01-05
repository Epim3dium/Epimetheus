#include "grid.hpp"
#include "RNG.h"
#include "particles.hpp"
#include "types.hpp"
namespace epi {
static auto shiftCyclic(size_t offset, const std::vector<vec2f>& points) {
    std::vector<vec2f> result(points.size());
    size_t index = 0;
    while(index != points.size()) {
        result[index] = points[(index + offset) % points.size()];
        index++;
    }
    return result;
}
static void mergeAdjacentRays(std::vector<vec2f>& points) {
    for(int i = 0; i < points.size(); i++) {

        vec2f& first = points[i];
        vec2f& mid   = points[(i + 1 ) % points.size()];
        vec2f& last  = points[(i + 2 ) % points.size()];

        auto& a0 = first;
        auto& b0 = mid;
        auto& a1 = mid;
        auto& b1 = last;
        if(normal(b0 - a0) == normal(b1 - a1)) {
            a1 = a0;
            points.erase(points.begin() + (i + 1 ) % points.size());
            i -= 1;
        }
    }
}
template<class Iter>
static std::vector<vec2f> smoothPointsPuecker(Iter points_begin, Iter points_end, float epsilon) {
    assert(points_begin < points_end);
    if(points_end - points_begin == 1) {
        return {*points_begin};
    }

    Iter cur_point;
    auto points_back = std::prev(points_end);

    float dmax = 0.f;
    auto ray_origin = *points_begin;
    auto ray_dir = *points_back - *points_begin;
    for(auto itr = std::next(points_begin); itr < points_back; itr++) {
        auto cur = *itr;
        auto closest = findClosestPointOnRay(ray_origin, ray_dir, cur);
        auto d= qlen(closest - *itr);
        if(d > dmax) {
            dmax = d;
            cur_point = itr;
        }
    }
    if(dmax > epsilon) {
        auto r1 = smoothPointsPuecker(points_begin, cur_point,  epsilon);
        auto r2 = smoothPointsPuecker(cur_point,    points_end, epsilon);
        r1.insert(r1.end(), r2.begin(), r2.end());
        return r1;
    }else {
        return {*points_begin, *points_back};
    }
}
static std::vector<vec2f> smoothPoints(const std::vector<vec2f>& points, float epsilon = 0.85f) {
    auto result = smoothPointsPuecker(points.begin(), points.end(), epsilon);
    if(result.size() < 3)
        return points;
    return result;
}
std::vector<vec2f> convertToPoints(const std::vector<std::pair<vec2f, vec2f>>& pairs) {
    std::vector<vec2f> result;
    auto last = pairs.back();
    for(auto p : pairs) {
        result.push_back(p.first);
    }
    return result;
}

std::vector<std::vector<vec2f>> Grid::m_extractPolygonPoints(AABB seg) {
    auto ray_groups = m_extractRayGroups(seg);
    std::vector<std::vector<vec2f>> point_islands;
    for(auto& group : ray_groups) {
        point_islands.push_back(convertToPoints(group));
    }
    for(auto& island : point_islands) {
        mergeAdjacentRays(island);
    }
    int smooth_steps = 3;
    for(int i = 0 ; i < smooth_steps; i++) {
        for(auto& island : point_islands) {
            auto shifted = shiftCyclic(island.size() / smooth_steps, island);
            island = smoothPoints(shifted);
        }
    }
    return point_islands;
}
ConcavePolygon Grid::m_extractPolygon(AABB seg) {
    std::vector<ConvexPolygon> all_polygons;
    auto all_points = m_extractPolygonPoints(seg);

    for(auto points : all_points) {
        auto triangles = triangulate(points);
        auto polygons = partitionConvex(triangles);
        all_polygons.insert(all_polygons.end(), polygons.begin(), polygons.end());
    }
    return  ConcavePolygon(all_polygons);
}
std::vector<std::vector<std::pair<vec2f, vec2f>>> Grid::m_extractRayGroups(AABB seg) {
    struct BorderCheckLine {
        vec2i local_border;
        vec2i check_dir;
        vec2i line_from;
        vec2i line_to;
    };
    static const std::vector<BorderCheckLine> hor_checks = {
        {vec2i{-1, 0},                vec2i{0, -1}, vec2i{1, 0}, vec2i{0, 0}},
        {vec2i{-1, SEGMENT_SIZE - 1}, vec2i{0, 1},  vec2i{0, 1}, vec2i{1, 1}},
    };
    static const std::vector<BorderCheckLine> ver_checks = {
        {vec2i{0, -1},                vec2i{-1, 0}, vec2i{0, 0}, vec2i{0, 1}},
        {vec2i{SEGMENT_SIZE - 1, -1}, vec2i{1, 0},  vec2i{1, 1}, vec2i{1, 0}},
    };
    static const vec2i invalid_pos = vec2i(-1, -1);

    std::vector<std::vector<std::pair<vec2f, vec2f>>> result;

    int base_x = seg.center().x / SEGMENT_SIZE;
    int base_y = seg.center().y / SEGMENT_SIZE;
    int xstart = base_x * SEGMENT_SIZE;
    int ystart = base_y * SEGMENT_SIZE;
    int xend = xstart + SEGMENT_SIZE;
    int yend = ystart + SEGMENT_SIZE;

    std::vector<std::vector<vec2i>> path(SEGMENT_SIZE + 1, std::vector<vec2i>(SEGMENT_SIZE + 1, invalid_pos));

    auto checkAndAddLines = [&](int xoff, int yoff, const std::vector<BorderCheckLine>& checks) {
        vec2f p(xoff + xstart, yoff + ystart);
        const auto& cell = get(p.x, p.y);
        if(isCellColliderEligable(cell)) {
            for(auto check : checks) {
                if(yoff == check.local_border.y || xoff == check.local_border.x) {
                    path[yoff + check.line_from.y][xoff + check.line_from.x] = check.line_to + vec2i(xoff, yoff);
                }
            }
        }else {
            for(auto check : checks) {
                const auto& cell = get(p.x + check.check_dir.x, p.y + check.check_dir.y);
                if(yoff != check.local_border.y && xoff != check.local_border.x 
                        && isCellColliderEligable(cell)) 
                {
                    path[yoff + check.line_to.y][xoff + check.line_to.x] = check.line_from + vec2i(xoff, yoff);
                }
            }
        }
    };
    auto findFirstInPath = [&]()-> vec2i {
        for(int y = 0; y < path.size(); y++) {
            for(int x = 0; x < path[y].size(); x++) {
                if(path[y][x] != invalid_pos) {
                    return {x, y};
                }
            }
        }
        return invalid_pos;
    };
    for (int yoff = 0; yoff < SEGMENT_SIZE; yoff++) {
        for (int xoff = 0; xoff < SEGMENT_SIZE; xoff++) {
            checkAndAddLines(xoff, yoff, hor_checks);
        }
    }
    for (int xoff = 0; xoff < SEGMENT_SIZE; xoff++) {
        for (int yoff = 0; yoff < SEGMENT_SIZE; yoff++) {
            checkAndAddLines(xoff, yoff, ver_checks);
        }
    }
    vec2i starting_pos;
    while((starting_pos = findFirstInPath()) !=  invalid_pos) {
        //new island
        result.push_back({});
        vec2i last = invalid_pos;
        vec2i coord = starting_pos;
        while(coord != last) {
            last = coord;
            coord = path[coord.y][coord.x];

            //mark as visited
            path[last.y][last.x] = invalid_pos;

            if(coord == invalid_pos)
                break;
            vec2f a = vec2f(coord) + vec2f(xstart, ystart);
            vec2f b = vec2f(last) + vec2f(xstart, ystart);
            result.back().push_back({b, a});
        }
    }
    return result;
}
void Grid::m_updateSegment(SegmentT seg, size_t index) {
    if (seg.max.x == -1) {
        return;
    }
    bool yinv = (last_tick_updated % 4 == 0 || last_tick_updated % 3 == 0);

    int yincr = (yinv ? 1 : -1);
    int ybegin = seg.bottom() - 1;
    int yend = seg.top() + 1;

    int xincr = (last_tick_updated % 2 == 0 ? 1 : -1);
    int xbegin = seg.left() - 1;
    int xend = seg.right() + 1;


    for (int y = (yinv ? ybegin : yend); y <= yend && y >= ybegin; y += yincr) {
        for (int x = (last_tick_updated % 2 == 0 ? xbegin : xend);
             x >= xbegin && x <= xend; x += xincr) {
            if (get(x, y).last_time_updated >= last_tick_updated)
                continue;
            world[m_idx(x, y)].last_time_updated = last_tick_updated;
            auto func =
                Cell::g_updates[static_cast<size_t>(world[m_idx(x, y)].type)];
            func(*this, {x, y});
        }
    }

    auto cur_changed = current_segments[index].hasColliderChanged;
    if((seg.hasColliderChanged && !cur_changed) || (last_tick_updated + index) % 10 == 0) {
        seg.hasColliderChanged = false;
        auto itr = terrain_objects.segment_colliders.begin();
        for(int i = 0; i < index; i++) itr++;
        auto shape = m_extractPolygon(seg);
        itr->collider.setShape(shape);
        itr->collider.time_immobile = 0.f;
        itr->transform.setPos(shape.getPos());
    }
}
void Grid::m_convertFloatingParticles(ParticleManager& manager) {
    for (auto seg : current_segments) {
        for (int y = seg.bottom(); y <= seg.top(); y++) {
            for (int x = seg.left(); x <= seg.right(); x++) {
                if (world[m_idx(x, y)].isFloating) {
                    auto cell = world[m_idx(x, y)];
                    cell.isFloating = false;
                    world[m_idx(x, y)] = Cell(eCellType::Air);
                    manager.add(vec2f(x + Particles::radius,
                                      y + Particles::radius),
                                cell, vec2f(0.f, -50000.f));
                }
            }
        }
    }
}

} // namespace epi
