#include "entity.hpp"
namespace epi {

uint64_t getNewEntityID() {
    static uint64_t  s_ID = 1;
    return s_ID++;
}
}
