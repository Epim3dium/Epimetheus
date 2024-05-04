#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include <_types/_uint64_t.h>
#include <functional>
struct Entity {
private:
    uint32_t id;
    static uint32_t getNextId() {
        static uint32_t s_id = 0;
        return s_id++;
    }
public:
    const char* name;
    inline const uint32_t operator()() const {
        return id;
    }
    inline bool operator<(const Entity& other) const {
        return id < other();
    }
    inline bool operator>(const Entity& other) const {
        return id > other();
    }
    inline bool operator==(const Entity& other) const {
        return id == other();
    }
    inline bool operator!=(const Entity& other) const {
        return id != other();
    }
    inline static Entity invalid() {
        static const Entity invalid("invalid Entity");
        return invalid;
    };
    Entity() : id(getNextId()), name("unnamed entity") {}
    Entity(const char* name_) : id(getNextId()), name(name_) {}
    Entity(const Entity& other) = default;
    Entity(Entity&& other) = default;
    Entity& operator=(const Entity& other) = default;
    Entity& operator=(Entity&& other) = default;
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
