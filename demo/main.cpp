#include "app.hpp"

using namespace epi;

using Transform::Position;
using Transform::Rotation;
using Transform::Scale;
using Transform::LocalTransform;
using Transform::GlobalTransform;
using Hierarchy::Parent;

void updateParentTransformByHierarchy(
    OwnerSlice<LocalTransform, GlobalTransform> slice,
    const Hierarchy::System& hierarchy, std::vector<size_t> index_list) 
{
    size_t to_update = 0;
    int iter = 0;
    for (auto to_update_index : index_list) {
        auto [e, my_trans, global_trans] = *std::next(slice.begin(), to_update_index);

        auto parent_maybe = hierarchy.try_cget<Hierarchy::Parent>(e);
        assert(parent_maybe.has_value());
        auto parent = *parent_maybe.value();

        if (parent == e)
            continue;

        const auto& global_trans_maybe = slice.try_get<GlobalTransform>(parent);
        if(!global_trans_maybe.has_value()) {
            continue;
        }
        global_trans = *global_trans_maybe.value();
        global_trans.combine(my_trans);
    }
}

struct System {
    Transform::System transform_sys;
    Hierarchy::System hierarchy_sys;
    Rigidbody::System rigidbody_sys;
    Collider::System collider_sys;
    Material::System material_sys;
    Constraint::System constraint_sys;
    PhysicsManager phy_man;
    Entity world = Entity("world");
    std::unordered_map<Entity, sf::Color> color_table;
    void add(Entity id, Transform::Position pos, Transform::Rotation r, Hierarchy::Parent parent,
             sf::Color color, std::vector<vec2f> model) {
        transform_sys.push_back(id, pos, r);
        hierarchy_sys.push_back(id, parent, Hierarchy::Children{});
        hierarchy_sys.try_get<Hierarchy::Children>(parent).value()->push_back(id);
        color_table[id] = color;
        
        auto area = epi::area(model);
        rigidbody_sys.push_back(id, Rigidbody::isStaticFlag(false), Rigidbody::Mass(area));
        material_sys.push_back(id);
        collider_sys.push_back(id, model);
    }
    void add(Entity id, Hierarchy::Parent parent,
             sf::Color color, std::vector<vec2f> points, bool isStatic = false) {
        auto pos = centerOfMass(points);
        for(auto& p : points) {
            p -= pos;
        }
        
        transform_sys.push_back(id, Position(pos), Rotation{0.f});
        hierarchy_sys.push_back(id, parent, Hierarchy::Children{});
        hierarchy_sys.try_get<Hierarchy::Children>(parent).value()->push_back(id);
        color_table[id] = color;
        auto area = epi::area(points);
        rigidbody_sys.push_back(id, Rigidbody::isStaticFlag(isStatic), Rigidbody::Mass(area));
        material_sys.push_back(id);
        collider_sys.push_back(id, points);
    }
    System() {
        hierarchy_sys.setDefault(Hierarchy::Parent(world));
        transform_sys.setDefault<Transform::LocalTransform>(Transform::LocalTransform::Identity);
        transform_sys.setDefault(Transform::Scale(vec2f(1.f, 1.f)));
        transform_sys.setDefault<Transform::GlobalTransform>(Transform::GlobalTransform::Identity);
        
        hierarchy_sys.push_back(world, {Entity::invalid()}, {});
        transform_sys.push_back(world, Position{}, Rotation{0.f});
    }
};


template<class T>
void lerpAny(T& first, const T other, float t) {
    first = first * t + other * (1.f - t);
}
template<>
void lerpAny<Entity>(Entity& first, Entity other, float t) {
    assert(first == other);
}
template <>
void lerpAny<Collider::ShapeModel>(Collider::ShapeModel&, Collider::ShapeModel, float) {}
template <>
void lerpAny<Collider::ShapePartitioned>(Collider::ShapePartitioned&, Collider::ShapePartitioned, float) {}
template <>
void lerpAny<Collider::ShapeTransformedPartitioned>(Collider::ShapeTransformedPartitioned&,
                                                    Collider::ShapeTransformedPartitioned, float) {}
template <>
void lerpAny<Collider::Mask>(Collider::Mask&, Collider::Mask, float) {}
template <>
void lerpAny<Collider::Tag>(Collider::Tag&, Collider::Tag, float) {}
template <>
void lerpAny<Transform::LocalTransform>(Transform::LocalTransform&, Transform::LocalTransform, float) {}
template <>
void lerpAny<Transform::GlobalTransform>(Transform::GlobalTransform&, Transform::GlobalTransform, float) {}
//sys0 has to be older
void lerp(double t, System& sys0, System& sys1) {
    auto lerpSys = [&](auto& sy0, auto& sy1) {
        auto slice0 = sy0.sliceAllOwner();
        auto slice1 = sy1.sliceAllOwner();
        auto itr1 = slice1.begin();
        while(itr1 != slice1.end()) {
            Entity entity = std::get<0U>(*itr1);
            if(!slice0.contains(entity)) {
                itr1++;
                continue;
            }
            auto itr0 = slice0.begin() + slice0.getIndex(entity).value();
            
            std::apply([&](auto&... values1) 
                {
            std::apply([&](auto&... values0) 
                {
                    (lerpAny(values0, values1, t), ...);
                }, *itr0);
            }, *itr1);
            
            itr1++;
        }
    };
    lerpSys(sys0.rigidbody_sys, sys1.rigidbody_sys);
    lerpSys(sys0.collider_sys, sys1.collider_sys);
    lerpSys(sys0.material_sys, sys1.material_sys);
    lerpSys(sys0.transform_sys, sys1.transform_sys);
}
class Demo : public Application {
public:
    System sys;

    Entity red = Entity("red");
    Entity magenta = Entity("magenta");
    Entity yellow = Entity("yellow");
    Entity green = Entity("green");
    Entity blue = Entity("blue");

    std::vector<vec2f> model_rect = {vec2f(30.f, 30.f), /* vec2f(40.f, 0.f), */    vec2f(30.f, -30.f)/* , vec2f(10.f, -37.f), vec2f(8.f, -39.f), vec2f(0.f, -40.f) */, vec2f(-30.f, -30.f), vec2f(-30.f, 30.f)};
    //green
    vec2f def_avg = {0, 0}; 
    //red
    vec2f model_avg = std::reduce(model_rect.begin(), model_rect.end()) / static_cast<float>(model_rect.size());
    //blue
    vec2f area_avg = centerOfMass(model_rect);
    //pos, pos + model_avg, pos + area_avg, model
    
    vec2f win_size = {800.f, 800.f};
    AABB big_aabb = AABB::CreateMinMax(vec2f(), win_size); 
    AABB small_aabb = AABB::CreateCenterSize(big_aabb.center(), big_aabb.size() * 0.9f); 
    std::vector<size_t> hierarchy_bfs;

    struct SaveData {
        double time_stamp;
        System world_state;
    };

    const float widget_line_width = 1.f;
    const float widget_shaft_length = 50.f;

    const float arrowhead_length = widget_shaft_length * 0.3f;
    const float arrowhead_width = arrowhead_length * 0.5f;
    const std::vector<vec2f> arrow_model = { 
        vec2f(0.f , -widget_shaft_length),
        vec2f(-widget_line_width, -0.f ),
        vec2f(-widget_line_width, -widget_shaft_length),
        vec2f(-arrowhead_width, -widget_shaft_length),
        vec2f(-0.f, -widget_shaft_length -arrowhead_length),
        vec2f(arrowhead_width, -widget_shaft_length),
        vec2f(widget_line_width , -widget_shaft_length),
        vec2f(widget_line_width , -0.f ),
        vec2f(-widget_line_width, -0.f ),
    };
    const float grabber_size = 7.f;
    const std::vector<vec2f> scaler_model = { 
        vec2f(0.f , -widget_shaft_length),
        vec2f(-widget_line_width, -0.f ),
        vec2f(-widget_line_width, -widget_shaft_length),
        vec2f(-grabber_size, -widget_shaft_length),
        vec2f(-grabber_size, -widget_shaft_length - grabber_size * 2.f),
        vec2f(grabber_size, -widget_shaft_length - grabber_size * 2.f),
        vec2f(grabber_size, -widget_shaft_length),
        // vec2f( , -shaft_length),
        vec2f(widget_line_width , -widget_shaft_length),
        vec2f(widget_line_width , -0.f ),
        vec2f(-widget_line_width, -0.f ),
    };
    struct {
        
        int editingFlag = 0;
        const int EDIT_POS = 1;
        const int EDIT_SCALE = 2;
        const int EDIT_ROTATION = 4;
        Entity id = Entity();
    }selection;
    template <class T>
    void edit(Entity id, sf::RenderWindow& window, std::vector<vec2f> widget_model,
              std::function<float(float, float, float)> feedbackx,
              std::function<float(float, float, float)> feedbacky, bool useRotation = false) {
        static bool lastTimeHeld = false;
        static int isHeldFlag = false;
        static const int AXIS_X = 1;
        static const int AXIS_Y = 2;
        
        auto position_maybe = sys.transform_sys.try_get<Transform::Position>(id);
        auto rotation_maybe = sys.transform_sys.try_get<Transform::Rotation>(id);
        auto& value_changed = sys.transform_sys.get<T>(id);
        if(!position_maybe.has_value() || !rotation_maybe.has_value()) {
            return;
        }
        auto pos = *position_maybe.value();
        auto rot = *rotation_maybe.value();
        const vec2f scale = {1.f, 1.f};
        sf::Transform YarrowTrans = sf::Transform().translate(pos).scale(scale);
        sf::Transform XarrowTrans = sf::Transform().translate(pos).rotate(90.f).scale(scale);
        if(useRotation) {
            XarrowTrans = sf::Transform().translate(pos).rotate(90.f + rot / 3.141f * 180.f).scale(scale);
            YarrowTrans = sf::Transform().translate(pos).rotate(rot / 3.141f * 180.f).scale(scale);
        }

        std::vector<sf::Vertex> vertex_buf;
        for(auto& p : widget_model) {
            vertex_buf.push_back({p, (isHeldFlag & AXIS_Y) ? sf::Color::White : sf::Color::Green});
        }
        sf::RenderStates state;

        state.transform = YarrowTrans;
        window.draw(&vertex_buf[0], widget_model.size(), sf::TriangleFan, state); 
        for(auto& p : vertex_buf) {
            p.color = (isHeldFlag & AXIS_X) ? sf::Color::White : sf::Color::Blue;
        }
        state.transform = XarrowTrans;
        window.draw(&vertex_buf[0], widget_model.size(), sf::TriangleFan, state); 

        //reacting to clicks
        if(!sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
            lastTimeHeld = false;
            isHeldFlag = 0;
            return;
        }
        vec2f mouse_pos = (vec2f)sf::Mouse::getPosition(window);
        static vec2f start_mouse_pos;
        static vec2f start_entity_value;
        if(!lastTimeHeld) {
            isHeldFlag = 0;
            start_mouse_pos = mouse_pos;
            start_entity_value = value_changed;
            isHeldFlag += isOverlappingPointPoly(mouse_pos, Transform::transformPoints(widget_model, YarrowTrans)) * 2;
            isHeldFlag += isOverlappingPointPoly(mouse_pos, Transform::transformPoints(widget_model, XarrowTrans));
        }
        lastTimeHeld = true;

        switch(isHeldFlag) {
            case AXIS_X:
                value_changed.x = feedbackx(start_entity_value.x, start_mouse_pos.x, mouse_pos.x);
                break;
            case AXIS_Y:
                value_changed.y = feedbacky(start_entity_value.y, start_mouse_pos.y, mouse_pos.y);
                break;
            case 0:
                break;
        }
    }
    void editRot(Entity id, sf::RenderWindow& window) {
        static bool lastTimeHeld = false;
        static bool isHeld = false;
        
        auto position_maybe = sys.transform_sys.try_get<Transform::Position>(id);
        auto rotation_maybe = sys.transform_sys.try_get<Transform::Rotation>(id);
        if(!position_maybe.has_value() || !rotation_maybe.has_value()) {
            return;
        }
        auto& pos = *position_maybe.value();
        auto& rot = *rotation_maybe.value();
        
        const int seg_count = 32U;
        const float radius = 50.f;
        const float ring_width = widget_line_width * 2.f;
        
        float angle_step = 2.f * M_PI / static_cast<float>(seg_count);
        float seg_size = angle_step * radius;
        sf::RectangleShape rs;
        vec2f size = {seg_size, ring_width};
        rs.setSize(size);
        rs.setOrigin(size / 2.f);
        rs.setFillColor(isHeld ? sf::Color::White : sf::Color::Red);
        for(int i = 0; i < seg_count; i++) {
            rs.setRotation(i * angle_step / M_PI * 180.f + 90.f);
            rs.setPosition(pos + rotate(vec2f(radius, 0.f), i * angle_step));
            window.draw(rs);
        }
        if(!sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
            lastTimeHeld = false;
            isHeld = false;
            return;
        }
        
        vec2f mouse_pos = (vec2f)sf::Mouse::getPosition(window);
        static vec2f start_mouse_pos;
        static float start_rotation;
        if(!lastTimeHeld) {
            bool overlappingOuter = isOverlappingPointCircle(mouse_pos, Circle(pos, radius + ring_width / 2.f));
            bool overlappingInner = isOverlappingPointCircle(mouse_pos, Circle(pos, radius - ring_width / 2.f));
            if(overlappingOuter && !overlappingInner) {
                isHeld = true;
            }
            start_mouse_pos = mouse_pos;
            start_rotation = rot;
        }
        lastTimeHeld = true;
        if(!isHeld) {
            return;
        }
        rot = start_rotation + angleAround(start_mouse_pos, pos, mouse_pos);
    }
    void editPos(Entity id, sf::RenderWindow& window) {
        auto feedback =
            [](float start_v, float start_mouse, float mouse) {
                return start_v + mouse - start_mouse;
            };
        edit<Transform::Position>(id, window, arrow_model, feedback, feedback); 
    }
    void editScale(Entity id, sf::RenderWindow& window) {
        auto feedbackx = 
            [](float start_v, float start_mouse, float mouse) {
                if(mouse - start_mouse > 0.f) {
                    return start_v + (mouse - start_mouse) / 10.f;
                }
                return start_v + (mouse - start_mouse) / 20.f;
            };
        auto feedbacky = 
            [](float start_v, float start_mouse, float mouse) {
                if(start_mouse - mouse > 0.f) {
                    return start_v + (start_mouse - mouse) / 10.f;
                }
                return start_v + (start_mouse - mouse) / 20.f;
            };
        edit<Transform::Scale>(id, window, scaler_model, feedbackx, feedbacky, true); 
    }
    
    std::vector<SaveData> saves;
    SaveData latest_save;
    const double save_interval = 0.125f;
    double current_time;
    
    std::vector<vec2f> creation_points;
    bool setup() override {
        setConstantFramerate(60);
        sys.add(yellow,  Parent{sys.world}, sf::Color::Yellow,  {big_aabb.bl(), big_aabb.br(), small_aabb.br(), small_aabb.bl()}, true);
        sys.add(magenta, Parent{sys.world}, sf::Color::Magenta, {big_aabb.tl(), big_aabb.tr(), small_aabb.tr(), small_aabb.tl()}, true);
        sys.add(red,     Parent{sys.world}, sf::Color::Red,     {big_aabb.bl(), big_aabb.tl(), small_aabb.tl(), small_aabb.bl()}, true);
        sys.add(green,   Parent{sys.world}, sf::Color::Green,   {big_aabb.br(), big_aabb.tr(), small_aabb.tr(), small_aabb.br()}, true);
        // sys.add(Entity(), "", Position({100.f, 100.f}), Rotation(0.f), sys.world, sf::Color::White, model_rect);
        hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
        
        saves.push_back({0.0, sys});
        
        ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true;

        addHook(sf::Event::MouseButtonPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.mouseButton.button == sf::Mouse::Left) {
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    creation_points.push_back((vec2f)mouse_pos);
                }
            });
        addHook(sf::Event::KeyPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::R) {
                    creation_points.clear();
                }
            });
        addHook(sf::Event::KeyPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::Enter) {
                    bool isStat = false;
                    if(event.key.shift) {
                        isStat =true;
                    }
                    sys.add(Entity(), sys.world, sf::Color::White, creation_points, isStat);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
                }
            });
        addHook(sf::Event::KeyPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::V) {
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    sys.add(Entity(), Position({static_cast<float>(mouse_pos.x), static_cast<float>(mouse_pos.y)}), Rotation(0.f), sys.world, sf::Color::White, model_rect);
                    // sys.rb_sys.get<Rigidbody::lockRotationFlag>(t) = true;
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
                }
            });
        addHook(sf::Event::KeyPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::B) {
                    auto m = model_rect;
                    for(auto& p : m) p *= 0.5f;
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    sys.add(Entity(), Position((vec2f)mouse_pos), Rotation(0.f), sys.world, sf::Color::White, m);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
                }
            });
        addHook(sf::Event::KeyPressed, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::C) {
                    auto m = model_rect;
                    for(auto& p : m) p *= 1.5f;
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    sys.add(Entity(), Position((vec2f)mouse_pos), Rotation(0.f), sys.world, sf::Color::White, m);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
                }
            });
        addHook(sf::Event::KeyReleased, 
            [&](sf::Event event, const sf::Window& window) {
                if(event.key.code == sf::Keyboard::Dash) {
                    sys = latest_save.world_state;
                }
            });
        return true;
    }
    void update(sf::Time deltaTime) override final {
        float fixedDeltaTime = 1.f / 60.f;
        // float fixedDeltaTime = delTtime.asSeconds();
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Dash)) {
            current_time -= fixedDeltaTime * 2.f; 
            while(current_time < saves.back().time_stamp) {
                latest_save = saves.back();
                sys = latest_save.world_state;
                saves.pop_back();
                EPI_LOG_DEBUG << "reloaded new save_state";
            }
            float lerp_time = std::clamp<float>(1.f - (latest_save.time_stamp - current_time) / save_interval, 0.f, 1.f);
            
            sys = latest_save.world_state;
            lerp(lerp_time, sys, saves.back().world_state);
            hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy_sys.sliceOwner<Parent>());
        }else {
            latest_save.world_state = sys;
            latest_save.time_stamp = current_time;
        }
        if(current_time - save_interval > saves.back().time_stamp) {
            EPI_LOG_DEBUG << "saved!";
            saves.push_back({current_time, sys});
        }
        current_time += fixedDeltaTime;

        // clear the window with black color
        
        Transform::updateLocalTransforms(sys.transform_sys.slice<Position, Rotation, Scale, LocalTransform>());
        
        updateParentTransformByHierarchy(
            sys.transform_sys.sliceOwner<LocalTransform, GlobalTransform>(),
            sys.hierarchy_sys, hierarchy_bfs);

        
        static ThreadPool tp;
        sys.phy_man.update(sys.transform_sys, sys.rigidbody_sys, sys.collider_sys, sys.material_sys, sys.constraint_sys, fixedDeltaTime /* delTtime.asSeconds() */, tp);
        
        {
            ImGui::Begin("settings");
            ImGui::Text("ObjectCount: %zu", sys.transform_sys.size());
            static constexpr size_t sample_size = 300U;
            static size_t cur_fps_idx = 0;
            static std::vector<double> FPS(sample_size);
            FPS[(cur_fps_idx++) % sample_size] = 1.0 / deltaTime.asSeconds();
            auto avg_fps = std::reduce(FPS.begin(), FPS.end()) / static_cast<double>(FPS.size());
            
            ImGui::Text("FPS: %f", avg_fps);
            
            ImGui::Dummy({});
            for (auto [e] : sys.hierarchy_sys.sliceOwner<>()) {
                ImGui::SameLine();
                if (ImGui::Button(e.name, {50.f, 20.f})) {
                    selection.editingFlag = 0;
                    selection.id = e;
                }
            }
            auto DFSpath = Hierarchy::getDFSIndexList(sys.hierarchy_sys.sliceAllOwner());
            for(auto idx : DFSpath) {
                auto [e] = *(sys.hierarchy_sys.sliceOwner<>().begin() + idx);
            }
            
            for (auto [e] : sys.hierarchy_sys.sliceOwner<>()) {
                if (selection.id != e) {
                    // if(sys.rb_sys.try_get<Rigidbody::isStaticFlag>(e).has_value())
                    //     *sys.rb_sys.try_get<Rigidbody::isStaticFlag>(e).value() = false;
                    continue;
                }
                ImGui::BeginChild("tab");
                
                ImGui::Text("%s", "settings");
            
                auto& scale = sys.transform_sys.get<Transform::Scale>(selection.id);
                ImGui::DragFloat2("scale", (float*)&scale, 0.05f);
                
                if(ImGui::Button("edit_s")) {
                    selection.editingFlag ^= selection.EDIT_SCALE;
                    selection.editingFlag &= selection.EDIT_SCALE;
                }
                
                auto& position = sys.transform_sys.get<Transform::Position>(selection.id);
                ImGui::DragFloat2("position", (float*)&position);
                
                if(ImGui::Button("edit_p")) {
                    selection.editingFlag ^= selection.EDIT_POS;
                    selection.editingFlag &= selection.EDIT_POS;
                }
                
                auto& rotation = sys.transform_sys.get<Transform::Rotation>(selection.id);
                ImGui::DragFloat("rotation", (float*)&rotation, 0.01f);
                
                if(ImGui::Button("edit_r")) {
                    selection.editingFlag = selection.EDIT_ROTATION * !(selection.editingFlag & selection.EDIT_ROTATION);
                }
                ImGui::EndChild();
            }
            ImGui::End();
        }
    }
    virtual void render(sf::RenderWindow& window) override final {
        std::vector<sf::Vector2f> positions;
        std::vector<Entity> ids;
        for (auto [e, global_trans] : sys.transform_sys.sliceOwner<GlobalTransform>()) {
            positions.push_back(global_trans.transformPoint(0.f, 0.f));
            ids.push_back(e);
        }

        sf::CircleShape cs;
        cs.setRadius(5.f);
        cs.setOrigin(5.f, 5.f);
        for (int i = 0; i < positions.size(); i++) {
            cs.setPosition(positions[i]);
            cs.setFillColor(sys.color_table[ids[i]]);
            window.draw(cs);
        }
        cs.setFillColor(sf::Color::Red);
        cs.setRadius(2.f);
        cs.setOrigin(2.f, 2.f);
        for (int i = 0; i < creation_points.size(); i++) {
            cs.setPosition(creation_points[i]);
            window.draw(cs);
        }
        cs.setRadius(2.f);
        cs.setOrigin(2.f, 2.f);
        cs.setFillColor(sf::Color::Green);
        Collider::calcParitionedShapes(sys.collider_sys.slice<Collider::ShapeModel, Collider::ShapePartitioned>());
        Collider::updatePartitionedTransformedShapes(
            sys.collider_sys.sliceOwner<Collider::ShapePartitioned, Collider::ShapeTransformedPartitioned>(),
            sys.transform_sys.slice<Transform::GlobalTransform>());
        
        for(auto [shape] : sys.collider_sys.slice<Collider::ShapeTransformedPartitioned>()) {
            sf::Vertex vert[2];
            static const sf::Color clr = sf::Color::White; 
            vert[0].color = clr;
            vert[1].color = clr;
            for(auto& convex : shape) {
                vert[0].position = convex.back();
                for(auto& point : convex) {
                    vert[1].position = point;
                    window.draw(vert, 2U, sf::Lines);
                    vert[0] = vert[1];
                }
            }
        }
        if(selection.editingFlag & selection.EDIT_POS) {
            editPos(selection.id, window);
        }
        if(selection.editingFlag & selection.EDIT_SCALE) {
            editScale(selection.id, window);
        }
        if(selection.editingFlag & selection.EDIT_ROTATION) {
            editRot(selection.id, window);
        }
    }
    Demo() : Application("demo", {800, 800}) {}
};

int main() {
    Demo demo;
    demo.run();
    return 0;
}
