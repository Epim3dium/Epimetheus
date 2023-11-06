#include <gtest/gtest.h>
#include <thread>
#define ANKERL_NANOBENCH_IMPLEMENT
#include "nanobench.h"
#include "core/group.hpp"
#include "RNG.h"

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

std::string gen_random(const int len, RNG& rng) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::string tmp_s;
    tmp_s.reserve(len);

    for (int i = 0; i < len; ++i) {
        tmp_s += alphanum[rng.Random<size_t>(0, sizeof(alphanum) - 1)];
    }
    
    return tmp_s;
}
void update_values(std::string& n, double& px, double& py, bool iss, double velx, double vely) {
    if(!iss) {
        px += 1.f / sqrt(velx);
        py += 1.f / sqrt(vely);
    }
    n += std::to_string(px) + ", " + std::to_string(py);
}
void update_values_small(double& px, double vely) {
    for(double i = 0; i < vely; i += 0.01) {
        px += sqrt(sin(i * i));
    }
}
TEST(GroupTest, GroupPerformanceTest) {
    RNG rng;
    enum class eVariables {
        NameString,
        PosxDouble,
        PosyDouble,
        isStaticBool,
        VelxDouble,
        VelyDouble,
    };

    struct Entity {
        int64_t dumb0;
        std::string Name;
        int64_t dumb1;
        double posx;
        int64_t dumb2;
        double posy;
        int64_t dumb3;
        bool isStatic;
        int64_t dumb4;
        double velx;
        int64_t dumb5;
        double vely;
        Entity(std::string s, double d1, double d2, bool b, double d3, double d4) 
            : posx(d1), posy(d2), isStatic(b), velx(d3), vely(d4) {}
    };
    std::vector<std::tuple<std::string, double, double, bool, double, double>> data;
    size_t iter_count = 500000;
    for(auto i = iter_count; i--;) {
        data.push_back({
            gen_random(10, rng), 
            rng.Random<double>(0.0, 1.0),
            rng.Random<double>(0.0, 1.0),
            rng.Random() > 0.5 ? true : false,
            rng.Random<double>(0.0, 10.0),
            rng.Random<double>(0.0, 1.0)
        });
    }
    Group<eVariables>::pointer group;
    {
        Group<eVariables>::Factory fac;
        fac.add<std::string>(eVariables::NameString );
        fac.add<double>(eVariables::PosxDouble );
        fac.add<double>(eVariables::PosyDouble );
        fac.add<bool>(eVariables::isStaticBool );
        fac.add<double>(eVariables::VelxDouble );
        fac.add<double>(eVariables::VelyDouble );
        group = fac.create();
    }
    std::vector<std::string>NameString ;
    std::vector<double     >PosxDouble;
    std::vector<double     >PosyDouble;
    std::vector<bool       >StaticBool;
    std::vector<double     >VelxDouble;
    std::vector<double     >VelyDouble;

    std::vector<Entity*> entities;
    for(auto& d : data ) {
        group->push_back(std::get<0>(d), std::get<1>(d), std::get<2>(d), std::get<3>(d), std::get<4>(d), std::get<5>(d));
        entities.push_back(new Entity{std::get<0>(d), std::get<1>(d), std::get<2>(d), std::get<3>(d), std::get<4>(d), std::get<5>(d)});
        NameString.push_back(std::get<0>(d));
        PosxDouble.push_back(std::get<1>(d));
        PosyDouble.push_back(std::get<2>(d));
        StaticBool.push_back(std::get<3>(d));
        VelxDouble.push_back(std::get<4>(d));
        VelyDouble.push_back(std::get<5>(d));
    }

    ankerl::nanobench::Bench().run("group_small", [&]() {
        group->update({eVariables::PosxDouble, eVariables::VelyDouble}, update_values_small);
    });

    ankerl::nanobench::Bench().run("entitiess_small", [&]() {
        for(auto e : entities) {
            update_values_small(e->posx, e->vely);
        }
    });
    ankerl::nanobench::Bench().run("vector_small", [&]() {
        for(size_t i = 0; i < entities.size(); i++) {
            update_values_small(PosxDouble[i], VelyDouble[i]);
        }
    });
}
