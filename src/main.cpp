#include <iostream>
#include <thread>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vulkan/vulkan.hpp>
#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>

#include "system.hpp"
#include "timer.h"
#include "RNG.h"


using namespace epi;

// template <class... Ts, std::size_t... Is, class Tuple>
// decltype( auto ) tie_from_specified( std::index_sequence<Is...>, Tuple& tuple )
// {
//     return std::tuple<Ts...>{ std::get<Is>( tuple )... };
// }
//
// template <class... Ts, class Tuple>
// decltype( auto ) tie_from( Tuple& tuple )
// {
//     return tie_from_specified<Ts...>( std::make_index_sequence<sizeof...( Ts )>{}, tuple );
// }
//
// template <class... Types>
// std::tuple<Types*...> tup() {
//     return {};
// }

float global = 0.f;
void update_func(float& f, int& i , char& c) {
    global += i / (int)c;
    global *= f;
}

int main(int argc, char **argv) {
    std::cin.get();
    RNG rng;
    SystemFactory fac;
    fac.add<float, int, char>();
    auto sys = fac.create();

    std::vector<float> vecf;
    std::vector<int> veci;
    std::vector<char> vecc;
    auto initSys = [&](int count) {
        sys->clear();
        for(int i = 0; i < count; i++)
            sys->push_back(rng.Random(0.f, 1.f), rng.Random(0, 1000), rng.Random('a', 'z'));
    };
    auto initVec = [&](int count) {
        vecf.clear();
        veci.clear();
        vecc.clear();
        for(int i = 0; i < count; i++) {
            vecf.push_back(rng.Random(0.f, 1.f));
            veci.push_back(rng.Random(0, 1000));
            vecc.push_back(rng.Random('a', 'z'));
        }
    };
    int iters = 10, elems = 10000 ;

    for(int i = 0; i < iters; i++) {
        initVec(elems);
        initSys(elems);
        {
             timer::scope timer("sys");
             for(size_t idx = 0; idx < elems; idx++) {
                 sys->update(update_func);
             }
        }
        {
            timer::scope time("v");
            for(size_t idx = 0; idx < elems; idx++) {
                update_func(vecf[idx], veci[idx], vecc[idx]);
            }
        }
        std::cout << "system: " << timer::Get("sys") << "\tvectors: " << timer::Get("v") << "\n";
        timer::Get("sys").reset();
        timer::Get("v").reset();
        std::cout << global << "\n";
    }
    return 0;
}
