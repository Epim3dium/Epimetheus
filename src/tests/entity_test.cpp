#include <gtest/gtest.h>
#include "scene/entity.hpp"
#include "debug/log.hpp"
using namespace epi;

TEST(EntityTest, BasicCreation) {
    auto world = Entities::create();

    auto player = Entities::create(world);
    auto sword = Entities::create(player);

    auto platform = Entities::create(world);

    auto maybe_vec =  Entities::getByID<Entities::ChildContainer>(eEntity::children_chdcontainer, player);
    //added sword
    ASSERT_TRUE(maybe_vec.has_value());
    auto& vec = maybe_vec.value().get();
    auto itr = std::find(vec.begin(), vec.end(), sword);
    ASSERT_NE(itr, vec.end());

    auto maybe_parent =  Entities::getByID<Entities::ID_t>(eEntity::parentID_IDt, player);
    ASSERT_TRUE(maybe_parent.has_value());
    ASSERT_EQ(maybe_parent->get(), world);
}
