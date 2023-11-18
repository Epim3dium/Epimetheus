#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "timer.h"
#include "templates/component_group.hpp"
#include "templates/primitive_wrapper.hpp"


using namespace epi;

#include <gtest/gtest.h>

struct Material {
    float sfric;
    float dfric;
    float bounce;
};
struct Velocity {
    float x;
    float y;
};
struct Decoy : PrimitiveWrapper<float> {};

int main(int argc, char** argv)
{
    auto g = ComponentGroup::Factory()
        .add<Material>()
        .add<Velocity>()
        .add<Decoy>().create();
    g->push_back(Entity::create(), Material{}, Velocity{21.f, 37.f},  Decoy());
    g->push_back(Entity::create(), Material{}, Velocity{6.f,  9.f},   Decoy());
    g->push_back(Entity::create(), Material{}, Velocity{3.f,  1.41f}, Decoy());

    typedef ComponentGroup::Form<Velocity> Wanted_T;
    auto mold = g->formulize<Velocity>();
    mold.forEach([](Entity id, Velocity& vel)->void {
        std::cout << id << "\t" << vel.x << " : " << vel.y << "\n";
    });
    std::cout << "\n\n";
    for(auto [id, vel] : mold) {
        std::cout << id << "\t" << vel.x << " : " << vel.y << "\n";
    }
    const auto& ref = mold;
    for(auto [id, vel] : ref) {
        std::cout << id << "\t" << vel.x << " : " << vel.y << "\n";
    }



    Log::ReportingLevel = LogLevel::WARNING;
    testing::InitGoogleTest(&argc, argv);
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
}
