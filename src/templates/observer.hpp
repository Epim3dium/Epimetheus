#ifndef EPI_OBSERVER_HPP
#define EPI_OBSERVER_HPP
#include <vector>
namespace epi {

namespace Signal {
//base data that is being sent between Subject and Observer
typedef std::string Event;

static constexpr const char* EventDestroyed = "destroyed";
static constexpr const char* EventChanged   = "changed";
static constexpr const char* EventInput   = "input";
//abstract class used for listening to Subject's signals
}

template<class EventType>
struct Subject;

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
#endif
