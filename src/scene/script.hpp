#ifndef EPI_SCRIPT_HPP
#define EPI_SCRIPT_HPP
#include "SFML/Graphics/RenderTarget.hpp"
#include "scene/component_group.hpp"
#include "scene/entity.hpp"
namespace epi {

enum class eScript {
    onUpdate_func,
    onRender_func,
};
class Scripts {
    ComponentGroup<eScript>::pointer group;
public:
    typedef std::function<void(Entities::ID_t)> onUpdate_t;
    typedef std::function<void(Entities::ID_t, sf::RenderTarget&)> onRender_t;
    void create(Entities::ID_t parent, onUpdate_t onUpdate, onRender_t onRender = nullptr);
    void create(Entities::ID_t parent, onRender_t onRender, onUpdate_t onUpdate = nullptr);
    void onUpdate();
    void onRender(sf::RenderTarget& rt);
    Scripts() : group(ComponentGroup<eScript>::Factory()
            .add<onUpdate_t>(eScript::onUpdate_func)
            .add<onRender_t>(eScript::onRender_func)
            .create()) {}
};

}
#endif // EPI_SCRIPT_HPP
