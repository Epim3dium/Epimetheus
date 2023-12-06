#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/System/Vector2.hpp"
#include "types.hpp"

namespace epi {

typedef sf::Vector2f vec2f;

struct TransformEvent {
    bool isPosChanged;
    bool isRotChanged;
    bool isScaleChanged;
};
class Transform : public Signal::Subject<TransformEvent> {
    vec2f _pos;
    vec2f _scale;
    float _rot;
public:
    vec2f getPos() const  {
        return this->_pos;
    }
    void setPos(vec2f v) {
        this->_pos = v;
        notify({true, false, false});
    }

    vec2f getScale() const {
        return _scale;
    }
    void setScale(vec2f v) {
        _scale = v;
        notify({false, false, true});
    }

    float getRot() const {
        return _rot;
    }
    void setRot(float r) {
        _rot = r;
        notify({false, true, false});
    }
    Transform() : _pos(0, 0), _scale(1.f, 1.f), _rot(0.f) {
    }
};

}
