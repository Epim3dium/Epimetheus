#include <gtest/gtest.h>
#include <unordered_set>
#include "core/group.hpp"
#include "core/primitive_wrapper.hpp"

using namespace epi;

struct position {
    float x;
    float y;
};
struct rotation { float val; };
struct scale {
    float x;
    float y;
};

TEST(group, adding_removing_accessing) {
    Group<position, rotation, scale> transforms;
    Entity player;
    Entity enemy1;
    Entity enemy2;
    Entity enemy3;
    transforms.push_back(player, {0.f, 0.f}, {0.f}, {1.f, 1.f});
    transforms.push_back(enemy1, {100.f, 100.f}, {90.f}, {2.f, 2.f});
    ASSERT_EQ(transforms.size(), 2U);
    transforms.push_back(enemy2, {200.f, 100.f}, {0.f}, {1.5f, 1.5f});
    ASSERT_EQ(transforms.size(), 3U);
    
    ASSERT_TRUE(transforms.getComponent<scale>(enemy2).has_value());
    auto scale2 = transforms.getComponent<scale>(enemy2).value();
    ASSERT_EQ(scale2->x, 1.5f);
    ASSERT_EQ(scale2->y, 1.5f);

    ASSERT_TRUE(transforms.erase(enemy1));
    ASSERT_FALSE(transforms.contains(enemy1));
    ASSERT_TRUE(transforms.contains(enemy2));
    ASSERT_TRUE(transforms.contains(player));
    ASSERT_FALSE(transforms.erase(enemy3));
    
    transforms.push_back(enemy3, {1000.f, 0.f}, {0.f}, {1.f, 1.f});
    ASSERT_EQ(transforms.size(), 3U);
}
TEST(group, group_iteration_and_slicing) {
    Group<position, rotation, scale> transforms;
    Entity player;
    Entity enemy1;
    Entity enemy2;
    Entity enemy3;
    const std::unordered_set<Entity> used_entities = {player, enemy1, enemy2, enemy3};
    const float default_rot = 0.f;
    transforms.push_back(player, {0.f, 0.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy1, {100.f, 100.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy2, {200.f, 100.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy3, {1000.f, 0.f}, {default_rot}, {1, 1});
    auto cur_set_check = used_entities;
    for(auto [e, pos, rot, scale] : transforms) {
        ASSERT_EQ(scale.x, 1.f);
        ASSERT_EQ(scale.y, 1.f);
        cur_set_check.erase(e);
    }
    ASSERT_EQ(cur_set_check.size(), 0U);

    auto rot_slice = transforms.slice<rotation>();
    cur_set_check = used_entities;
    for(auto [e, rot] : rot_slice) {
        ASSERT_EQ(rot.val, default_rot);
        cur_set_check.erase(e);
    }
    ASSERT_EQ(cur_set_check.size(), 0U);
}
struct velocity {
    float x;
    float y;
};
struct angular_velocity {
    float val;
};
TEST(group, group_slicing_overlap) {
    Group<position, rotation, scale> transforms;
    Entity player;
    Entity enemy1;
    Entity enemy2;
    Entity enemy3;
    Entity error_causing_box;
    const float default_rot = 0.f;
    transforms.push_back(player, {0.f, 0.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy1, {100.f, 100.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy2, {200.f, 100.f}, {default_rot}, {1.f, 1.f});
    transforms.push_back(enemy3, {1000.f, 0.f}, {default_rot}, {1, 1});
    Group<velocity, angular_velocity> rigidbodies;
    rigidbodies.push_back(enemy1, {10.f, 10.f}, {0.f});
    rigidbodies.push_back(enemy2, {-10.f, 10.f}, {1.f});
    rigidbodies.push_back(error_causing_box, {0.f, 0.f}, {0.f});
    
    size_t counter = 0;
    for(auto [entity, vel, ang_vel] : rigidbodies) {
        if(!transforms.contains(entity)) {
            ASSERT_EQ(entity, error_causing_box);
            continue;
        }
        counter ++;
    }
    ASSERT_EQ(counter, 2U);
}
struct EntityParent {
    static Entity null;
    Entity val;
    EntityParent(const Entity& e) : val(e) {}
    EntityParent() : val(null) {}
};
Entity EntityParent::null;
struct global_position : public position {
    global_position& operator=(const position& pos) {
        this->x = pos.x;
        this->y = pos.y;
        return *this;
    }
    
};
TEST(group, group_reverse_iterating) {
    Group<position, global_position, EntityParent> hierarchy;
    Entity world;
    Entity player;
    Entity sword;
    Entity enemy1;
    Entity enemy2;
    hierarchy.push_back(enemy2, {-100, 0}, {}, {world});
    hierarchy.push_back(sword, {50, 50}, {}, {player});
    hierarchy.push_back(enemy1, {200, 100}, {}, {world});
    hierarchy.push_back(player, {100, 0}, {}, {world});
    hierarchy.push_back(world, {100, 100}, {}, EntityParent());
    for(auto it = hierarchy.rbegin(); it != hierarchy.rend(); it++) {
        auto [entity, pos, gpos, parent] = *it;
        if(parent.val == EntityParent::null) {
            gpos = pos;
            continue;
        }
        auto parents_position = hierarchy.getComponent<global_position>(parent.val);
        ASSERT_TRUE(parents_position.has_value());
        gpos = {parents_position.value()->x + pos.x, parents_position.value()->y + pos.y};
    }
    auto sword_pos = hierarchy.getComponent<global_position>(sword).value();
    ASSERT_EQ(sword_pos->x, 50 + 100 + 100);
    ASSERT_EQ(sword_pos->y, 50 + 100);
}
