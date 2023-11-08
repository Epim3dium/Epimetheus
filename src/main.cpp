#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "core/group.hpp"
#include "core/component_group.hpp"
#include "timer.h"


using namespace epi;

template<class Enum>
struct GlobalGetter {
    static typename ComponentGroup<Enum>::pointer group;
};
template<class Enum>
typename ComponentGroup<Enum>::pointer GlobalGetter<Enum>::group = {};

enum class eTBD {
};

#include <gtest/gtest.h>
int main(int argc, char** argv)
{
    GlobalGetter<eTBD>::group = ComponentGroup<eTBD>::Factory().create();

    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
