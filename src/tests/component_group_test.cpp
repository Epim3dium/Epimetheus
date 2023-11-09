
#include <gtest/gtest.h>
#include "core/component_group.hpp"
#include "core/entity.hpp"
using namespace epi;

enum class Player {
    HP,
    Attack,
    Name
};

TEST(ComponentGroupTest, PushingUpdatingErasing) {
    ComponentGroup<Player>::pointer comp_group;
    {
        ComponentGroup<Player>::Factory fac;
        fac.add<float>(Player::HP );
        fac.add<float>(Player::Attack );
        fac.add<std::string>(Player::Name );
        comp_group = fac.create();
    }
    comp_group->push_back<float, float, std::string>(Entities::getNewID(), 100.f, 1.f, "first_player");
    comp_group->push_back<float, float, std::string>(Entities::getNewID(), 0.f, 10.f, "second_player");
    comp_group->push_back<float, float, std::string>(Entities::getNewID(), 33.f, 5.f,  "third_player");
    std::vector<EntityID> ids_to_delete;
    comp_group->updateWithID({Player::HP, Player::Name},
        [&](EntityID id, float hp, std::string name) {
            if(hp == 0) {
                ids_to_delete.push_back(id);
            }
        });
    for(auto i : ids_to_delete) {
        comp_group->eraseByID(i);
    }
    comp_group->updateWithID({Player::Name, Player::HP},
        [&](EntityID id, std::string name, float hp) {
            ASSERT_NE(hp, 0.f);
        });
}

enum class Rigidbody {
    floatVelx,
    floatVely,
};
enum class Collider {
    boolIsStatic,
    StringMaterial,
};
TEST(ComponentGroupTest, UpdateMatching) {
    ComponentGroup<Rigidbody>::pointer rigidbody_group;
    ComponentGroup<Collider>::pointer  collider_group;
    {
        ComponentGroup<Rigidbody>::Factory fac;
        fac.add<float>(Rigidbody::floatVelx);
        fac.add<float>(Rigidbody::floatVely);
        rigidbody_group = fac.create();
    }
    {
        ComponentGroup<Collider>::Factory fac;
        fac.add<bool>(Collider::boolIsStatic);
        fac.add<std::string>(Collider::StringMaterial);
        collider_group = fac.create();
    }
    auto player   = Entities::getNewID();
    auto platform = Entities::getNewID();
    auto trigger  = Entities::getNewID();
    auto enemy    = Entities::getNewID();

    rigidbody_group->push_back(player, 100.f, 100.f);
    rigidbody_group->push_back(platform, 0.f, 0.f);
    rigidbody_group->push_back(enemy, -100.f, 200.f);

    collider_group->push_back(player, false, std::string("skin"));
    collider_group->push_back(platform, true, std::string("metal"));
    collider_group->push_back(trigger, true, std::string("none"));
    collider_group->push_back(enemy, false, std::string("skin"));
    updateMatching<Rigidbody, Collider>(*rigidbody_group.get(), *collider_group.get(),
        {Rigidbody::floatVelx, Rigidbody::floatVely}, {Collider::boolIsStatic},
        [](float& velx, float& vely, bool isStatic) {
            if(isStatic) {
                velx = -1.f;
                vely = -1.f;
            }else {
                vely += 10.f;
            }
        });
    ASSERT_EQ(rigidbody_group->getByID<float>(Rigidbody::floatVely, player), 110.f);
    ASSERT_EQ(rigidbody_group->getByID<float>(Rigidbody::floatVelx, platform), -1.f);
    ASSERT_EQ(rigidbody_group->getByID<float>(Rigidbody::floatVely, enemy), 210.f);

}
