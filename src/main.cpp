#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>

#include "io/app.hpp"
#include "math/math_defs.hpp"
#include "math/math_func.hpp"
#include "templates/group.hpp"
#include "timer.h"
#include "scene/component_group.hpp"


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
struct Decoy {
    float x;
    float y;
};
// Make a FOREACH macro
#define FE_0(WHAT, WHAT_LAST)
#define FE_1(WHAT, WHAT_LAST, X     ) WHAT_LAST(X, first  ) 
#define FE_2(WHAT, WHAT_LAST, X, ...) WHAT(X, second )FE_1(WHAT, WHAT_LAST, __VA_ARGS__)
#define FE_3(WHAT, WHAT_LAST, X, ...) WHAT(X, third  )FE_2(WHAT, WHAT_LAST, __VA_ARGS__)
#define FE_4(WHAT, WHAT_LAST, X, ...) WHAT(X, fourth )FE_3(WHAT, WHAT_LAST, __VA_ARGS__)
#define FE_5(WHAT, WHAT_LAST, X, ...) WHAT(X, fifth  )FE_4(WHAT, WHAT_LAST, __VA_ARGS__)
#define FE_6(WHAT, WHAT_LAST, X, ...) WHAT(X, sixth  )FE_5(WHAT, WHAT_LAST, __VA_ARGS__)
#define FE_7(WHAT, WHAT_LAST, X, ...) WHAT(X, seventh)FE_6(WHAT, WHAT_LAST, __VA_ARGS__)

#define FEL_0(WHAT)
#define FEL_1(WHAT, X     ) WHAT(X, first  ) 
#define FEL_2(WHAT, X, ...) WHAT(X, second ), FEL_1(WHAT, __VA_ARGS__)
#define FEL_3(WHAT, X, ...) WHAT(X, third  ), FEL_2(WHAT, __VA_ARGS__)
#define FEL_4(WHAT, X, ...) WHAT(X, fourth ), FEL_3(WHAT, __VA_ARGS__)
#define FEL_5(WHAT, X, ...) WHAT(X, fifth  ), FEL_4(WHAT, __VA_ARGS__)
#define FEL_6(WHAT, X, ...) WHAT(X, sixth  ), FEL_5(WHAT, __VA_ARGS__)
#define FEL_7(WHAT, X, ...) WHAT(X, seventh), FEL_6(WHAT, __VA_ARGS__)
//... repeat as needed

#define GET_MACRO(_0,_1,_2,_3,_4,_5, _6, _7,NAME,...) NAME 
#define FOR_EACH(action, last_action,...) \
  GET_MACRO(_0,__VA_ARGS__,FE_7,FE_6,FE_5,FE_4,FE_3,FE_2,FE_1,FE_0)(action, last_action,__VA_ARGS__)
#define FOR_EACH_LIST(action,...) \
  GET_MACRO(_0,__VA_ARGS__,FEL_7,FEL_6,FEL_5,FEL_4,FEL_3,FEL_2,FEL_1,FEL_0)(action,__VA_ARGS__)

// Example
// Some actions
#define SPAN_DECL(T, name) std::span<T> name;

#define LIST_ARG(T, name) name

#define LIST_ARG_OF_IDX_i(T, name) name[i]

#define LIST_TYPE(T, name) T

#define INIT_ARG(T, name) std::span<T> name##_

#define INIT_VAL(T, name) name(name##_)

#define TEMPLATE_ARG(T, name) class T

// Helper function
#define SPANS(...) FOR_EACH(SPAN_DECL, SPAN_DECL,__VA_ARGS__)


#define LIST_ARGS_OF_IDX_i(...) FOR_EACH_LIST(LIST_ARG_OF_IDX_i, __VA_ARGS__)
#define LIST_ARGS(...) FOR_EACH_LIST(LIST_ARG, __VA_ARGS__)
#define LIST_TYPES(...) FOR_EACH_LIST(LIST_TYPE,__VA_ARGS__)
#define INIT_ARGS(...) FOR_EACH_LIST(INIT_ARG,__VA_ARGS__)
#define INIT_VALS(...) FOR_EACH_LIST(INIT_VAL,__VA_ARGS__)
#define TEMPLATE_ARGS(...) FOR_EACH_LIST(TEMPLATE_ARG,__VA_ARGS__)


template<class ...Types>
struct Complex {
};
#define COMPLEX_ANY(...)\
template<TEMPLATE_ARGS(__VA_ARGS__)>\
struct Complex<LIST_TYPES(__VA_ARGS__)> {\
    SPANS(__VA_ARGS__);\
    size_t m_size;\
    Complex(INIT_ARGS(__VA_ARGS__), size_t size)  : INIT_VALS(__VA_ARGS__), m_size(size) {}\
    template<class ...ArgT>\
    void m_for_each(std::function<void(ArgT...)>&& update_func, size_t start, size_t end) {\
        for(size_t i = start; i < std::min(m_size, end); i++) {\
            update_func(LIST_ARGS_OF_IDX_i(__VA_ARGS__));\
        }\
    }\
    template<class Func>\
    void for_each(Func&& update_func, size_t start = 0, size_t end = INFINITY) {\
        m_for_each(std::function(std::forward<Func>(update_func)), start, end);\
    }\
}
COMPLEX_ANY(T1);
COMPLEX_ANY(T1, T2);
COMPLEX_ANY(T1, T2, T3);
COMPLEX_ANY(T1, T2, T3, T4);
COMPLEX_ANY(T1, T2, T3, T4, T5);
COMPLEX_ANY(T1, T2, T3, T4, T5, T6);
COMPLEX_ANY(T1, T2, T3, T4, T5, T6, T7);

int main(int argc, char** argv)
{
    std::vector<char> c_vec;
    c_vec.push_back('a');
    c_vec.push_back('b');
    c_vec.push_back('c');
    std::vector<float> f_vec;
    f_vec.push_back(69);
    f_vec.push_back(420);
    f_vec.push_back(3141);
    Complex<char, float> c(std::span<char>(c_vec), std::span<float>(f_vec), c_vec.size());
    c.for_each([](char c, float f) {
            std::cout << c << " " << f << "\n";
            });

    uint64_t id = 0;
    auto g = ComponentGroup::Factory().add<Material>().add<Velocity>().add<Decoy>().create();
    g->push_back(id++, Material{}, Velocity{21.f, 37.f},  Decoy());
    g->push_back(id++, Material{}, Velocity{6.f,  9.f},   Decoy());
    g->push_back(id++, Material{}, Velocity{3.f,  1.41f}, Decoy());

    auto mold = g->mold<Velocity>();
    mold.for_each([](uint64_t id, Velocity& vel)->void {
        std::cout << vel.x << " : " << vel.y << "\n";
    });

    Log::ReportingLevel = LogLevel::WARNING;
    testing::InitGoogleTest(&argc, argv);
    int err;
    if((err = RUN_ALL_TESTS())) {
        return err;
    }
}
