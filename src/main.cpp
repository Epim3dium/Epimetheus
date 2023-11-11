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
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
}
