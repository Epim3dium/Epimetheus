#include <iostream>
#include <thread>
#include <SFML/Graphics.hpp>
#include <unordered_set>

#include "core/group.hpp"
#include "core/primitive_wrapper.hpp"
using namespace epi;


struct Position : public sf::Vector2f{
};
struct Rotation {
    float val = 0.f;
};
struct Scale : public sf::Vector2f {
};
struct LocalTransform : public sf::Transform {};
struct GlobalTransform : public sf::Transform {};

struct Parent : public Entity {};
struct Children : public std::vector<Entity> {};

typedef Group<Position, Rotation, Scale, LocalTransform, GlobalTransform> TransformGroup;
typedef Group<Parent, Children> HierarchyGroup;
void updateLocalTransformsByOrder(Slice<Entity, Position, Rotation, Scale, LocalTransform> splice) {
    for(auto [e, pos, rot, scale, trans] : splice) {
        trans = { sf::Transform::Identity};
        trans.scale(scale);
        trans.translate(pos);
        trans.rotate(rot.val);
    }
}
void updateParentTransformByOrder(Slice<Entity, LocalTransform, GlobalTransform> splice, const HierarchyGroup& hierarchy) {
    for(auto [e, my_trans, global_trans] : splice) {
        auto parent_maybe = hierarchy.getComponent<Parent>(e);
        assert(parent_maybe.has_value());
        auto parent = *parent_maybe.value();
        
        if(parent == e)
            continue;
        
        auto global_trans_maybe = splice.getComponent<GlobalTransform>(parent);
        assert(global_trans_maybe.has_value());
        global_trans = *global_trans_maybe.value();
        global_trans.combine(my_trans);
    }
}

int main()
{
    TransformGroup transforms;
    HierarchyGroup hierarchy;
    Entity world;
    
    std::get<Parent>(hierarchy.default_values) = {world};
    std::get<LocalTransform>(transforms.default_values) = LocalTransform{sf::Transform::Identity};
    std::get<Scale>(transforms.default_values) = {{1.f, 1.f}};
    std::get<GlobalTransform>(transforms.default_values) = GlobalTransform{sf::Transform::Identity};
    hierarchy.push_back(world);
    
    Entity red;
    Entity magenta;
    Entity green;
    Entity cyan;
    Entity blue;
    transforms.push_back(world, Position{}, Rotation{0.f} );
    transforms.push_back(red, Position{{100.f, 100.f}}, Rotation{45.f});
    transforms.push_back(magenta, Position{{100.f, 100.f}});
    transforms.push_back(green, Position{{400.f, 100.f}} );
    transforms.push_back(blue, Position{{600.f, 100.f}} );
    for(auto e : {red, green, blue}) {
        hierarchy.push_back(e);
        hierarchy.getComponent<Children>(world).value()->push_back(e);
    }
    hierarchy.push_back(magenta, Parent{red});
    hierarchy.getComponent<Children>(red).value()->push_back(magenta);
    
    std::unordered_map<Entity, sf::Color> color_table;
    color_table[red] = sf::Color::Red;
    color_table[magenta] = sf::Color::Magenta;
    color_table[green] = sf::Color::Green;
    color_table[blue] = sf::Color::Blue;
    
    // create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "My window");

    // run the program as long as the window is open
    while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // clear the window with black color
        window.clear(sf::Color::Black);
        updateLocalTransformsByOrder(transforms.slice<Position, Rotation, Scale, LocalTransform>());
        updateParentTransformByOrder(transforms.slice<LocalTransform, GlobalTransform>(), hierarchy);
        
        std::vector<sf::Vector2f> positions;
        std::vector<Entity> ids;
        for(auto [e, global_trans] : transforms.slice<GlobalTransform>()) {
            positions.push_back(global_trans.transformPoint(1.f, 1.f));
            ids.push_back(e);
        }

        sf::CircleShape cs;
        cs.setRadius(5.f);
        cs.setOrigin(5.f, 5.f);
        for(int i = 0; i < positions.size(); i++) {
            cs.setPosition(positions[i]);
            cs.setFillColor(color_table[ids[i]]);
            window.draw(cs);
        }

        // draw everything here...
        // window.draw(...);

        // end the current frame
        window.display();
    }

    return 0;
}
