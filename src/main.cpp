#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "core/group.hpp"
#include "core/component_group.hpp"
#include "timer.h"


using namespace epi;


#include <gtest/gtest.h>
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
