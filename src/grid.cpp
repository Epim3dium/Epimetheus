#include "grid.hpp"
#include "RNG.h"
#include "particles.hpp"
namespace epi {
static void mergeAdjacentRays(std::vector<std::pair<vec2f, vec2f>>& ray_set) {
    for(int i = 0; i < ray_set.size() - 1; i++) {
        auto& a0 = ray_set[i].first;
        auto& b0 = ray_set[i].second;
        auto& a1 = ray_set[i + 1].first;
        auto& b1 = ray_set[i + 1].second;
        if(normal(b0 - a0) == normal(b1 - a1)) {
            a1 = a0;
            ray_set.erase(ray_set.begin() + i);
            i--;
        }
    }
    std::cerr << "size: " << ray_set.size() << "\n";
}
std::vector<std::vector<std::pair<vec2f, vec2f>>> Grid::m_extractPolygon(AABB seg) {
    auto ray_sets = m_extractRayGroups(seg);
    for(auto& set : ray_sets) {
        mergeAdjacentRays(set);
    }
    return ray_sets;
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
        if(get(p.x, p.y).getPropery().state == eState::Solid) {
            for(auto check : checks) {
                if(yoff == check.local_border.y || xoff == check.local_border.x) {
                    path[yoff + check.to.y][xoff + check.to.x] = check.from + vec2i(xoff, yoff);
                }
            }
        }else {
            for(auto check : checks) {
                if(yoff != check.local_border.y && xoff != check.local_border.x 
                        && get(p.x + check.check_dir.x, p.y + check.check_dir.y).getPropery().state == eState::Solid) 
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
void Grid::m_updateSegment(AABB seg) {
    if (seg.max.x == -1) {
        return;
    }
    bool yinv = (last_tick_updated % 4 == 0 || last_tick_updated % 3 == 0);
    int yincr = (yinv ? 1 : -1);
    int ybegin = seg.bottom() - 1;
    int yend = seg.top() + 1;

    segment_outlines.push_back(m_extractPolygon(seg));
    for (int y = (yinv ? ybegin : yend); y <= yend && y >= ybegin; y += yincr) {
        int xincr = (last_tick_updated % 2 == 0 ? 1 : -1);
        int xbegin = seg.left() - 1;
        int xend = seg.right() + 1;
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
