#include <iostream>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vulkan/vulkan.hpp>
#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>

#include "system.hpp"


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

void update_func(const float& f, int& i , char& c) {
    std::cout << f << i << c << std::endl;
}
int main() {
    SystemFactory fac;
    fac.add<float>();
    fac.add<int>();
    fac.add<char>();
    auto sys = fac.create();
    sys->push_back(21.f, 3, '7');
    sys->push_back(4.f, 2, '0');

    // std::tuple<float*, int*, char*> t;
    // sys->  updateTuple<decltype(t), float, int, char>(0, 0, sys->m_buffers.data(), t);
    // *std::get<int*>(t) = 1000.;
    //
    // std::cout << sys->get<float>(0);
    // std::cout << sys->get<int>(0);
    // std::cout << sys->get<char>(0);

    sys->update(&update_func);

    // int x{ 2 };
    // int y{ 3 };
    // int z{ 4 };
    // std::tuple<int*, int*, int*> g19( &x, &y, &z );
    //
    // auto t0{ tie_from_expand<int&, int&, int&>( g19 ) };
    // std::cout << std::get<0>(t0);
    // std::cout << std::get<1>(t0);
    // std::cout << std::get<2>(t0);

    
    return 0;
}
