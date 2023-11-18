#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include <_types/_uint64_t.h>
#include <vector>

namespace epi {


class Entity {
    uint32_t m_id;
    static uint32_t getNextID() {
        static uint32_t s_id = 0;
        return s_id++;
    }
    Entity() : m_id(getNextID()) {}
public:
    operator uint32_t() const {
        return m_id;
    }
    static Entity create() {
        return Entity();
    }

};


} // namespace epi
namespace std {
    template<>
    struct hash<epi::Entity> {
        inline size_t operator()(const epi::Entity& x) const {
            // size_t value = your hash computations over x
            return x;
        }
    };
}
#endif // EPI_ENTITY_HPP
