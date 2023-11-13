#ifndef EPI_DEBUG_SHAPES_HPP
#define EPI_DEBUG_SHAPES_HPP
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/ConvexShape.hpp"
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/System/Vector2.hpp"
#include "scene/component_group.hpp"
#include "scene/entity.hpp"
#include "transform.hpp"
namespace epi {

enum class eDebugShape {
    shape_sf,
};
class DebugShapes {
    ComponentGroup<eDebugShape>::pointer m_group;

public:
    void render(sf::RenderTarget&, const Transforms& trans);

    void create(Entities::ID_t parent, std::vector<sf::Vector2f> points,
                sf::Color color = sf::Color::White);
    void createCircle(Entities::ID_t parent, float radius,
                sf::Color color = sf::Color::White, size_t vert_count = 16U);
    void erase(Entities::ID_t id);

    template <class T>
    inline std::optional<std::reference_wrapper<T>>
    getByID(eDebugShape variable, Entities::ID_t id) {
        return std::move(m_group->getByID<T>(variable, id));
    }
    DebugShapes()
        : m_group(ComponentGroup<eDebugShape>::Factory()
                      .add<sf::ConvexShape>(eDebugShape::shape_sf)
                      .create()) {}
};

} // namespace epi
#endif // EPI_DEBUG_SHAPES_HPP
