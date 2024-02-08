#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector2.hpp"
#include "math/types.hpp"

namespace epi {

typedef sf::Vector2f vec2f;

struct TransformEvent {
    bool isPosChanged;
    bool isRotChanged;
    bool isScaleChanged;
};
class Transform  {
    vec2f _pos;
    vec2f _scale;
    float _rot;
public:
    vec2f getPos() const  {
        return this->_pos;
    }
    void setPos(vec2f v) {
        this->_pos = v;
    }

    vec2f getScale() const {
        return _scale;
    }
    void setScale(vec2f v) {
        _scale = v;
    }

    float getRot() const {
        return _rot;
    }
    void setRot(float r) {
        _rot = r;
    }
    Transform() : _pos(0, 0), _scale(1.f, 1.f), _rot(0.f) {
    }
};

}
