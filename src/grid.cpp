#include "grid.hpp"
#include "RNG.h"
#include "particles.hpp"
#include "types.hpp"
namespace epi {
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
            i--;
        }
    }
}
std::vector<bool> findEdges(const std::vector<vec2f>& points, AABB seg) {
    std::vector<bool> result;
    for(auto p : points) {
        if(p.x == seg.left() || p.x == seg.right() || p.y == seg.bottom() || p.y == seg.top()) {
            result.push_back(true);
        }else {
            result.push_back(false);
        }
    }
    return result;
}
static std::vector<vec2f> smoothAdjacentRays(const std::vector<vec2f>& points, AABB seg) {
    std::vector<vec2f> new_set;
    auto isEdge = findEdges(points, seg);
    for(int i = 0; i < points.size(); i++) {
        vec2f first = points[i];
        vec2f cur = points[(i + 1) % points.size()];
        if(isEdge[(i + 1) % points.size()] || isEdge[i]) {
            new_set.push_back(first);
        } else {
            new_set.push_back((first + cur) * 0.5f);
        }
    }
    return new_set;
}
std::vector<vec2f> convertToPoints(const std::vector<std::pair<vec2f, vec2f>>& pairs) {
    std::vector<vec2f> result;
    for(auto p : pairs) {
        result.push_back(p.first);
    }
    return result;
}

std::vector<std::vector<vec2f>> Grid::m_extractPolygonPoints(AABB seg) {
    auto ray_sets = m_extractRayGroups(seg);
    std::vector<std::vector<vec2f>> point_sets;
    for(auto& set : ray_sets) {
        point_sets.push_back(convertToPoints(set));
    }
    for(auto& set : point_sets) {
        mergeAdjacentRays(set);
    }
    int smooth_steps = 0;
    for(int i = 0 ; i < smooth_steps; i++) {
        for(auto& set : point_sets) {
            set = smoothAdjacentRays(set, seg);
        }
        for(auto& set : point_sets) {
            mergeAdjacentRays(set);
        }
    }
    return point_sets;
}
ConcavePolygon Grid::m_extractPolygon(AABB seg) {
    std::vector<ConvexPolygon> all_polygons;
    for(auto points : m_extractPolygonPoints(seg)) {
        auto triangles = toTriangles(points);
        auto polygons = toBiggestConvexPolygons(triangles);
        all_polygons.insert(all_polygons.end(), polygons.begin(), polygons.end());
    }
    return  ConcavePolygon(all_polygons);
}
std::vector<std::vector<std::pair<vec2f, vec2f>>> Grid::m_extractRayGroups(AABB seg) {
    struct BorderCheckLine {
        vec2i local_border;
        vec2i check_dir;
        vec2i from;
        vec2i to;
    };
    static const std::vector<BorderCheckLine> hor_checks = {
        {vec2i{-1, 0},                vec2i{0, -1}, vec2i{1, 0}, vec2i{0, 0}},
        {vec2i{-1, SEGMENT_SIZE - 1}, vec2i{0, 1},  vec2i{0, 1}, vec2i{1, 1}},
    };
    static const std::vector<BorderCheckLine> ver_checks = {
        {vec2i{0, -1},                vec2i{-1, 0}, vec2i{0, 0}, vec2i{0, 1}},
        {vec2i{SEGMENT_SIZE - 1, -1}, vec2i{1, 0},  vec2i{1, 1}, vec2i{1, 0}},
    };
    std::vector<std::vector<std::pair<vec2f, vec2f>>> result;

    int base_x = seg.center().x / SEGMENT_SIZE;
    int base_y = seg.center().y / SEGMENT_SIZE;
    int xstart = base_x * SEGMENT_SIZE;
    int ystart = base_y * SEGMENT_SIZE;
    int xend = xstart + SEGMENT_SIZE;
    int yend = ystart + SEGMENT_SIZE;

    const vec2i invalid_pos = vec2i(-1, -1);
    std::vector<std::vector<vec2i>> path(SEGMENT_SIZE + 1, std::vector<vec2i>(SEGMENT_SIZE + 1, invalid_pos));

    auto checkAndAddLines = [&](int xoff, int yoff, const std::vector<BorderCheckLine>& checks) {
        vec2f p(xoff + xstart, yoff + ystart);
        const auto& cell = get(p.x, p.y);
        if(isCellColliderEligable(cell)) {
            for(auto check : checks) {
                if(yoff == check.local_border.y || xoff == check.local_border.x) {
                    path[yoff + check.to.y][xoff + check.to.x] = check.from + vec2i(xoff, yoff);
                }
            }
        }else {
            for(auto check : checks) {
                const auto& cell = get(p.x + check.check_dir.x, p.y + check.check_dir.y);
                if(yoff != check.local_border.y && xoff != check.local_border.x 
                        && isCellColliderEligable(cell)) 
                {
                    path[yoff + check.from.y][xoff + check.from.x] = check.to + vec2i(xoff, yoff);
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
        result.push_back({});
        vec2i last = invalid_pos;
        vec2i coord = starting_pos;
        bool invert = false;
        while(coord != last) {
            last = coord;
            coord = path[coord.y][coord.x];
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
    if(seg.hasColliderChanged)
        segment_outlines[index] = m_extractPolygon(seg);

    for (int y = (yinv ? ybegin : yend); y <= yend && y >= ybegin; y += yincr) {
        for (int x = (last_tick_updated % 2 == 0 ? xbegin : xend);
             x >= xbegin && x <= xend; x += xincr) {
            if (get(x, y).last_time_updated >= last_tick_updated)
                continue;
            world[m_idx(x, y)].last_time_updated = last_tick_updated;
            world[m_idx(x, y)].isFloating = false;
            auto func =
                Cell::g_updates[static_cast<size_t>(world[m_idx(x, y)].type)];
            func(*this, {x, y});
        }
    }
}
void Grid::convertFloatingParticles(ParticleManager& manager) {
    for (auto seg : current_segments) {
        for (int y = seg.bottom(); y <= seg.top(); y++) {
            for (int x = seg.left(); x <= seg.right(); x++) {
                if (world[m_idx(x, y)].isFloating) {
                    auto cell = world[m_idx(x, y)];
                    cell.isFloating = false;
                    world[m_idx(x, y)] = Cell(eCellType::Air);
                    manager.add(vec2f(x + ParticleGroup::radius,
                                      y + ParticleGroup::radius),
                                cell);
                }
            }
        }
    }
}

} // namespace epi
