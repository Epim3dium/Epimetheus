#include "script.hpp"
using namespace epi;
void Scripts::create(Entities::ID_t parent, onUpdate_t onUpdate, onRender_t onRender) {
    group->push_back<onUpdate_t, onRender_t>(parent, onUpdate, onRender);
}
void Scripts::create(Entities::ID_t parent, onRender_t onRender, onUpdate_t onUpdate) {
    create(parent, onUpdate, onRender);
}
void Scripts::onUpdate() {
    group->updateWithID({eScript::onUpdate_func}, 
        [](Entities::ID_t id, onUpdate_t func) {
            if(func)
                func(id);
        });
}
void Scripts::onRender(sf::RenderTarget& window) {
    group->updateWithID({eScript::onRender_func}, 
        [&window](Entities::ID_t id, onRender_t func) {
            if(func)
                func(id, window);
        });
}
