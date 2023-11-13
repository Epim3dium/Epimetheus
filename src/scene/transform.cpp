#include "transform.hpp"

using namespace epi;

// enum class eTransform {
//     position_vec2f,
//     rotation_float,
//     scale_vec2f,
//     parent_GlobalPosition_vec2f,
//     parent_GlobalRotation_float,
//     parent_GlobalScale_vec2f,
//     transform_sf,
//     parent_GlobalTransform_sf,
// };
void Transforms::create(Entities::ID_t owner, sf::Vector2f pos, float rot, sf::Vector2f scale) {
    sf::Transform trans = sf::Transform::Identity;
    trans.translate(pos);
    trans.rotate(rot);
    trans.scale(scale);
    m_group->push_back<sf::Vector2f, float, sf::Vector2f, bool, sf::Transform>(owner, pos, rot, scale, false, trans); 
}
void Transforms::erase(Entities::ID_t id) {
    m_group->eraseByID(id);
}

void Transforms::update() {
    m_group->update({eTransform::position_vec2f, eTransform::rotation_float, eTransform::scale_vec2f, eTransform::transform_sf}, 
        [](const sf::Vector2f& pos, float rot, const sf::Vector2f& scale, sf::Transform& trans) {
            trans = sf::Transform::Identity;
            trans.translate(pos);
            trans.rotate(rot);
            trans.scale(scale);
        });
}
