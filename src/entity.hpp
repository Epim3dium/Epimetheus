#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include <_types/_uint64_t.h>
#include <functional>
struct Entity {
private:
    const uint32_t id;
    static uint32_t getNextId() {
        static uint32_t s_id = 0;
        return s_id++;
    }
public:
    inline const uint32_t operator()() const {
        return id;
    }
    inline bool operator==(const Entity& other) const {
        return id == other();
    }
    inline bool operator!=(const Entity& other) const {
        return id != other();
    }
    Entity() : id(getNextId()) {}
};
template<>
struct std::hash<Entity>
{
    std::size_t operator()(const Entity& e) const noexcept
    {
        return e(); // or use boost::hash_combine
    }
};
#endif // EPI_ENTITY_HPP
