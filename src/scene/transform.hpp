#ifndef EPI_TRANSFORM_HPP
#define EPI_TRANSFORM_HPP
#include "SFML/Graphics/Transform.hpp"
#include "SFML/System/Vector2.hpp"
#include "scene/component_group.hpp"
#include "scene/entity.hpp"
namespace epi {

enum class eTransform {
    position_vec2f,
    rotation_float,
    scale_vec2f,
    parent_id,
    transform_sf,
};
class DebugShapes;

class Transforms {
    ComponentGroup<eTransform>::pointer m_group;
public:
    void create(Entities::ID_t parent, sf::Vector2f pos, float rot, sf::Vector2f scale);
    void erase(Entities::ID_t id);

    void update();
    template <class T>
    inline std::optional<std::reference_wrapper<T>>
    getByID(eTransform variable, Entities::ID_t id) {
        return std::move(m_group->getByID<T>(variable, id));
    }
    Transforms() : m_group(ComponentGroup<eTransform>::Factory()
            .add<sf::Vector2f>(eTransform::position_vec2f)
            .add<float>(eTransform::rotation_float)
            .add<sf::Vector2f>(eTransform::scale_vec2f)
            .add<bool>(eTransform::parent_id)
            .add<sf::Transform>(eTransform::transform_sf)
            .create()){}
    friend DebugShapes;
};

}
#endif  //EPI_TRANSFORM_HPP
