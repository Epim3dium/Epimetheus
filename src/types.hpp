#pragma once
#include <cmath>
#include <iostream>
#include <math.h>
#include <vector>
#include <set>

#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "math/math_defs.hpp"

#include "SFML/Graphics.hpp"
#include "math/math_func.hpp"

namespace epi {
typedef sf::Color Color;

namespace PastelColor {
    const Color bg =     Color(0xa89984ff);
    const Color bg1 =    Color(0xa89984ff);
    const Color bg2 =    Color(0xbdae93ff);
    const Color bg3 =    Color(0xebdbb2ff);
    const Color bg4 =    Color(0x1e3a4cff);
    const Color Red =    Color(0x9d0006ff);
    const Color Green =  Color(0x79740eff);
    const Color Yellow = Color(0xb57614ff);
    const Color Blue =   Color(0x076678ff);
    const Color Purple = Color(0x8f3f71ff);
    const Color Aqua =   Color(0x427b58ff);
    const Color Orange = Color(0xaf3a03ff);
    const Color Gray =   Color(0x928374ff);
};

struct Circle;
class ConvexPolygon;

struct AABB {
    vec2f min;
    vec2f max;

    vec2f bl() const {
        return min;
    }
    vec2f br() const {
        return vec2f(max.x, min.y);
    }
    vec2f tl() const {
        return vec2f(min.x, max.y);
    }
    vec2f tr() const {
        return max;
    }

    vec2f center() const {
        return min + (max-min) / 2.f;
    }
    float right() const {
        return max.x;
    };
    float left() const {
        return min.x;
    };
    float bottom() const {
        return min.y;
    };
    float top() const {
        return max.y;
    };
    vec2f size() const {
        return max - min;
    }
    void setCenter(vec2f c) {
        auto t = size();
        min = c - t / 2.f;
        max = c + t / 2.f;
    }
    void expandToContain(sf::Vector2f point) {
        min.x = std::fmin(min.x, point.x);
        min.y = std::fmin(min.y, point.y);
        max.x = std::fmax(max.x, point.x);
        max.y = std::fmax(max.y, point.y);
    }
    void setSize(vec2f s) {
        auto t = center();
        min = t - s / 2.f;
        max = t + s / 2.f;
    }
    AABB combine(AABB val) {
        val.min.x = std::min(min.x, val.min.x);
        val.min.y = std::min(min.y, val.min.y);
        val.max.x = std::max(max.x, val.max.x);
        val.max.y = std::max(max.y, val.max.y);
        return val;
    }
    friend void draw(sf::RenderWindow& rw, const AABB& aabb, Color clr);
    friend void drawFill(sf::RenderTarget& rw, const AABB& aabb, Color clr);
    friend void drawOutline(sf::RenderTarget& rw, const AABB& aabb, Color clr);

    static AABB CreateMinMax(vec2f min, vec2f max);
    static AABB CreateCenterSize(vec2f center, vec2f size);
    static AABB CreateMinSize(vec2f min, vec2f size);
    static AABB CreateFromCircle(const Circle& c);
    static AABB CreateFromPolygon(const ConvexPolygon& p);
    static AABB CreateFromVerticies(const std::vector<vec2f>& verticies);
};
struct Ray {
    vec2f pos;
    vec2f dir;

    float length() const {
        return epi::length(dir);
    }
    Ray() {}
    static Ray CreatePoints(vec2f a, vec2f b);
    static Ray CreatePositionDirection(vec2f p, vec2f d);
};
struct Triangle {
    vec2f a;
    vec2f b;
    vec2f c;
};
std::vector<ConvexPolygon> triangulate(const std::vector<vec2f>& points);
std::vector<ConvexPolygon> partitionConvex(const std::vector<ConvexPolygon>& polygons);
struct Circle {

    vec2f pos;
    float radius;
    Circle(vec2f p = vec2f(0, 0), float r = 1.f) : pos(p), radius(r) {}
};
class ConvexPolygon {
    std::vector<vec2f> points;
    std::vector<vec2f> model;
    float rotation;
    vec2f pos;
    vec2f scale = {1, 1};
    void m_updatePoints() {
        for(size_t i = 0; i < model.size(); i++) {
            const auto& t = model[i];
            points[i].x = (t.x * cosf(rotation) - t.y * sinf(rotation)) * scale.x;
            points[i].y = (t.x * sinf(rotation) + t.y * cosf(rotation)) * scale.y;
            points[i] += pos;
        }
    }
    static std::vector<vec2f> m_avgPoints(std::vector<vec2f> model) {
        vec2f avg = vec2f(0, 0);
        for(auto& p : model) {
            avg += p;
        }
        avg /= (float)model.size();
        for(auto& p : model) {
            p -= avg;
        }
        return model;
    }
public:
    float getRot() const {
        return rotation;
    }
    void setRot(float r) {
        rotation = r;
        m_updatePoints();
    }
    vec2f getPos() const {
        return pos;
    }
    void setPos(vec2f v) {
        pos = v;
        m_updatePoints();
    }
    void setScale(vec2f s) {
        scale = s;
        m_updatePoints();
    }
    vec2f getScale() const {
        return scale;
    }
    const std::vector<vec2f>& getVertecies() const {
        return points;
    }
    const std::vector<vec2f>& getModelVertecies() const {
        return model;
    }
    ConvexPolygon() {}
    ConvexPolygon(vec2f pos_, float rot_, const std::vector<vec2f>& model_) : points(model_.size(), vec2f(0, 0)), model(model_), rotation(rot_), pos(pos_) {
        std::sort(model.begin(), model.end(), [](vec2f a, vec2f b) {
            return atan2(a.y, a.x) < atan2(b.y, b.x);
        });
        m_updatePoints();
        //m_avgPoints(model);
    }

    static ConvexPolygon CreateRegular(vec2f pos, float rot, size_t count, float dist);
    static ConvexPolygon CreateFromAABB(const AABB& aabb);
    static ConvexPolygon CreateFromPoints(std::vector<vec2f> verticies);

    friend void draw(sf::RenderWindow& rw, const ConvexPolygon& poly, Color clr);
    friend void drawFill(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr);
    friend void drawOutline(sf::RenderTarget& rw, const ConvexPolygon& poly, Color clr);
};
struct ConcavePolygon {
private:
    std::vector<ConvexPolygon> m_model;
    std::vector<ConvexPolygon> m_polygons;
    vec2f pos;
    float rotation = 0.f;
    void m_updatePolys() {
        for(int i = 0; i < m_model.size(); i++) {
            m_polygons[i].setRot(m_model[i].getRot() + rotation);
        }
        for(int i = 0; i < m_model.size(); i++) {
            m_polygons[i].setPos(rotateVec(m_model[i].getPos(), rotation) + pos);
        }
    }
public:
    float getRot() const {
        return rotation;
    }
    void setRot(float r) {
        rotation = r;
        m_updatePolys();
    }
    vec2f getPos() const {
        return pos;
    }
    void setPos(vec2f v) {
        pos = v;
        m_updatePolys();
    }
    const std::vector<ConvexPolygon>& getModelPolygons() const {
        return m_model;
    }
    const std::vector<ConvexPolygon>& getPolygons() const {
        return m_polygons;
    }
    ConcavePolygon(std::vector<ConvexPolygon> polygons);

};


vec2f operator* (vec2f a, vec2f b);
class Tag {
    std::set<std::string> _tags;
public:
    inline std::vector<std::string> getList() const {
        return std::vector<std::string>(_tags.begin(), _tags.end());
    }
    inline void add(std::string t) {
        _tags.insert(t);
    }
    inline void remove(std::string t) {
        _tags.erase(t);
    }
    inline bool contains(std::string t) const {
        return _tags.contains(t);
    }
    inline bool operator==(std::string t) const {
        return contains(t);
    }
    inline bool operator==(const Tag& t) const {
        std::vector<std::string> common_data;
        set_intersection(_tags.begin(), _tags.end(), t._tags.begin(), t._tags.end(), std::back_inserter(common_data));
        return common_data.size() != 0;
    }
    inline bool operator!=(const char* t) const {
        return !(*this == t);
    }
    inline size_t size() const { return _tags.size(); };
    Tag() {}
    Tag(std::initializer_list<std::string> inits) : _tags(inits) {}
};
namespace Signal {
//base data that is being sent between Subject and Observer
typedef std::string Event;

static constexpr const char* EventDestroyed = "destroyed";
static constexpr const char* EventChanged   = "changed";
static constexpr const char* EventInput   = "input";

template<class EventType>
struct Subject;

//abstract class used for listening to Subject's signals
template<class EventType>
class Observer {
    std::vector<Subject<EventType>*> _subjects_observed;
    void _removeSubject(const Subject<EventType>* s) {
        auto itr =std::find(_subjects_observed.begin(), _subjects_observed.end(), s);
        if(itr != _subjects_observed.end()) {
            _subjects_observed.erase(itr);
        }
    }
public:
    virtual void onNotify(EventType event) = 0;
    virtual ~Observer() {
        for(auto s : _subjects_observed) {
            s->removeObserver(this);
        }
    }
    friend Subject<EventType>;
};

//class used for being able to notify other observers
template<class EventType>
class Subject {
private:
    std::vector<Observer<EventType>*> _observers;
public:
    void addObserver(Observer<EventType>* observer) {
        _observers.push_back(observer);
        observer->_subjects_observed.push_back(this);
    }
    void removeObserver(Observer<EventType>* observer) {
        auto itr = std::find(_observers.begin(), _observers.end(), observer);
        assert(itr != _observers.end());
        _observers.erase(itr);
    }

    void notify(EventType event) {
        for (auto o : _observers) {
            o->onNotify(event);
        }
    }
    virtual ~Subject() {
        for(auto o : _observers) {
            o->_removeSubject(this);
        }
    }
};
}
}
