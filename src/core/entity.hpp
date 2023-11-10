#ifndef EPI_ENTITY_HPP
#define EPI_ENTITY_HPP
#include <_types/_uint64_t.h>
#include <vector>
namespace epi {

template<class Enum>
class ComponentGroup;

enum class eEntity {
    ID_EntityID,
    parentID_EntityID,
    children_vectorEntityID,
};
class Entities {
public:
    typedef uint64_t IDtype;
    typedef std::vector<IDtype> ChildContainer;
private:
    static IDtype m_getNewID();
public:
    static ComponentGroup<eEntity>& group();
    //returns EntityID of entity created
    static IDtype create(IDtype parent = 0);
};

}
#endif // EPI_ENTITY_HPP
