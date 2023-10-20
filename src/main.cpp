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
struct OtherSystem {
    virtual void update() = 0;
};
struct OtherSysttemImpl : public OtherSystem {
    std::vector<std::tuple<float, int, char>*> objects;
    void update() override {
        for(auto& o : objects) {
        }
    }
};

void update_func(float f, int i , const char& c) {
    // global += i * c;
    // global *= f;
    std::cout << f << i << c << "\n";
}

int main(int argc, char **argv) {
    SystemFactory fac;
    fac.add<float, int, char>();
    auto sys = fac.create();
     sys->push_back(2.1f, 3, '7');
     sys->push_back(4.f, 2, '0');
     sys->push_back(100.f, 100, 'x');
     sys->update(update_func);
 }
//
//        RNG rng;
//     std::vector<float> vecf;
//     std::vector<int> veci;
//     std::vector<char> vecc;
//     std::vector<std::tuple<float, int, char>*> vec_aloc;
//     auto initAloc = [&](int count) {
//         vec_aloc.clear();
//         for(int i = 0; i < count; i++) {
//             vec_aloc.push_back(new std::tuple<float, int, char>(rng.Random(0.f, 1.f), rng.Random(0, 1000), rng.Random('a', 'z')));
//         }
//     };
//     auto initSys = [&](int count) {
//         sys->clear();
//         for(int i = 0; i < count; i++)
//             sys->push_back(rng.Random(0.f, 1.f), rng.Random(0, 1000), rng.Random('a', 'z'));
//     };
//     auto initVec = [&](int count) {
//         vecf.clear();
//         veci.clear();
//         vecc.clear();
//         for(int i = 0; i < count; i++) {
//             vecf.push_back(rng.Random(0.f, 1.f));
//             veci.push_back(rng.Random(0, 1000));
//             vecc.push_back(rng.Random('a', 'z'));
//         }
//     };
//     int iters = 5, elems = 100000 ;
//
//     std::cin.get();
//     initVec(elems);
//     initSys(elems);
//     initAloc(elems);
//     for(int i = 0; i < iters; i++) {
//         {
//              timer::scope timer("mem");
//              for(size_t idx = 0; idx < elems; idx++) {
//                  update_func(std::get<float>(*vec_aloc[idx]), std::get<int>(*vec_aloc[idx]), std::get<char>(*vec_aloc[idx]));
//              }
//         }
//         {
//              timer::scope timer("sys");
//              sys->update(update_func);
//         }
//         {
//             timer::scope time("v");
//             for(size_t idx = 0; idx < elems; idx++) {
//                 update_func(vecf[idx], veci[idx], vecc[idx]);
//             }
//         }
//         std::cout << timer::Get("sys") << timer::Get("v") << timer::Get("mem") << "\n";
//         timer::Get("sys").reset();
//         timer::Get("mem").reset();
//         timer::Get("v").reset();
//         std::cout << global << "\n";
//     }
//     return 0;
// }
