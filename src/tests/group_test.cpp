#include <gtest/gtest.h>
#include "core/group.hpp"

using namespace epi;

void side_effect_function(const float& velx, float vely, float& result) {
    result = velx * vely;
}
enum class eVelocity {
    velx,
    velz,
    vely,
    weird_product,
};

// Demonstrate some basic assertions.
TEST(GroupTest, CreationSameTypeComponents) {
    Group<eVelocity>::pointer group;
    {
        Group<eVelocity>::Factory fac;

        fac.add<float>(eVelocity::velx);
        fac.add<float>(eVelocity::vely);
        fac.add<float>(eVelocity::weird_product);
        group = fac.create();
    }
    group->push_back(21.f, 37.f, 0.f);
    group->push_back(6.f, 9.f, 0.f);
    group->update({eVelocity::velx, eVelocity::vely, eVelocity::weird_product}, side_effect_function);
    // Expect equality.
    EXPECT_EQ(group->get<float>(eVelocity::weird_product , 0), 777);
    EXPECT_EQ(group->get<float>(eVelocity::weird_product , 1), 54);
}

enum class eCollider {
    Magnitude,
    isStatic,
    Offset,
};
TEST(GroupTest, ErasingComponentsAndLambdas) {
    Group<eCollider>::pointer group;
    {
        Group<eCollider>::Factory fac;

        fac.add<bool>(eCollider::isStatic);
        fac.add<float>(eCollider::Magnitude);
        fac.add<float>(eCollider::Offset);
        group = fac.create();
    }
    group->push_back(false, 10.f,   0.f);
    group->push_back(true,  20.f,   0.f);
    group->push_back(true, 200.f,  6969.f);
    group->push_back(false, 1000.f, 2112.f);
    group->update({eCollider::isStatic, eCollider::Magnitude, eCollider::Offset}, 
        [](bool isStatic, float mag, float& offset) {
            if(isStatic)
                return;
            offset = mag * isStatic;
        }
    );
    group->erase(1U);
    // Expect equality.
    EXPECT_EQ(group->get<float>(eCollider::Offset , 2), 6969.f);

    group->erase(0U);
    EXPECT_EQ(group->get<float>(eCollider::Offset , 0), 6969.f);
}
