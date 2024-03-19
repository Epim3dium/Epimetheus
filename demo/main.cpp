#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>
#include <unordered_set>

#include "core/hierarchy.hpp"
#include "core/group.hpp"
#include "debug/log.hpp"
#include "physics/collider.hpp"
#include "physics/physics_manager.hpp"
#include "physics/rigidbody.hpp"
#include "templates/primitive_wrapper.hpp"
#include "imgui-SFML.h"
#include "imgui.h"
#include "imgui_internal.h"
using namespace epi;

bool SliderScalar2D(char const* pLabel, float* fValueX, float* fValueY,
                    const float fMinX, const float fMaxX, const float fMinY,
                    const float fMaxY, float const fZoom /*= 1.0f*/);
bool InputVec2(char const* pLabel, ImVec2* pValue, ImVec2 const vMinValue,
               ImVec2 const vMaxValue, float const fScale /*= 1.0f*/);

using Transform::Position;
using Transform::Rotation;
using Transform::Scale;
using Transform::LocalTransform;
using Transform::GlobalTransform;
using Hierarchy::Parent;

typedef Group<Position, Rotation, Scale, LocalTransform, GlobalTransform>
    TransformGroup;
void updateParentTransformByHierarchy(
    OwnerSlice<LocalTransform, GlobalTransform> slice,
    const Hierarchy::System& hierarchy, std::pair<int, std::vector<size_t>> layer_info) 
{
    size_t to_update = 0;
    int iter = 0;
    for (auto to_update_index : layer_info.second) {
        auto [e, my_trans, global_trans] = *std::next(slice.begin(), to_update_index);

        auto parent_maybe = hierarchy.try_cget<Hierarchy::Parent>(e);
        assert(parent_maybe.has_value());
        auto parent = *parent_maybe.value();

        if (parent == e)
            continue;

        const auto& global_trans_maybe = slice.get<GlobalTransform>(parent);
        global_trans = global_trans_maybe;
        global_trans.combine(my_trans);
    }
}

struct System {
    Transform::System transforms;
    Hierarchy::System hierarchy;
    Rigidbody::System rb_sys;
    Collider::System col_sys;
    Material::System mat_sys;
    PhysicsManager phy_man;
    Entity world;
    std::unordered_map<Entity, sf::Color> color_table;
    std::unordered_map<Entity, std::string> name_table;
    void add(Entity id, std::string name, Transform::Position pos, Transform::Rotation r, Hierarchy::Parent parent,
             sf::Color color, std::vector<vec2f> model) {
        name_table[id] = name;
        transforms.push_back(id, pos, r);
        hierarchy.push_back(id, parent, Hierarchy::Children{});
        hierarchy.try_get<Hierarchy::Children>(parent).value()->push_back(id);
        color_table[id] = color;
        
        auto area = epi::area(model);
        rb_sys.push_back(id, Rigidbody::isStaticFlag(false), Rigidbody::Mass(area));
        mat_sys.push_back(id);
        col_sys.push_back(id, model);
    }
    void add(Entity id, std::string name, Hierarchy::Parent parent,
             sf::Color color, std::vector<vec2f> points, bool isStatic = false) {
        name_table[id] = name;
        auto pos = centerOfMass(points);
        for(auto& p : points) {
            p -= pos;
        }
        
        transforms.push_back(id, Position(pos), Rotation{0.f});
        hierarchy.push_back(id, parent, Hierarchy::Children{});
        hierarchy.try_get<Hierarchy::Children>(parent).value()->push_back(id);
        color_table[id] = color;
        auto area = epi::area(points);
        rb_sys.push_back(id, Rigidbody::isStaticFlag(isStatic), Rigidbody::Mass(area));
        mat_sys.push_back(id);
        col_sys.push_back(id, points);
    }
    System() {
        hierarchy.setDefault(Hierarchy::Parent(world));
        transforms.setDefault<Transform::LocalTransform>(Transform::LocalTransform::Identity);
        transforms.setDefault(Transform::Scale(vec2f(1.f, 1.f)));
        transforms.setDefault<Transform::GlobalTransform>(Transform::GlobalTransform::Identity);
        hierarchy.push_back(world, Parent{world});
        transforms.push_back(world, Position{}, Rotation{0.f});
    }
};
#include "imgui.h" // necessary for ImGui::*, imgui-SFML.h doesn't include imgui.h

#include "imgui-SFML.h" // for ImGui::SFML::* functions and SFML-specific overloads

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>


vec2f getCenterOfMass(std::vector<vec2f> model) {
    vec2f sum_avg = {0, 0};
    float sum_weight = 0.f;
    auto prev = model.back();
    for (auto next : model) {
        float area_step = abs(cross(prev, next))/2.f;
        sum_weight += area_step;
        sum_avg += (next + prev) / 3.f * area_step;
        prev = next;
    }
    return sum_avg / sum_weight;
}

int main() {
    System sys;

    Entity red;
    Entity magenta;
    Entity yellow;
    Entity green;
    Entity blue;

    std::vector<vec2f> model_rect = {vec2f(30.f, 30.f), /* vec2f(40.f, 0.f), */    vec2f(30.f, -30.f)/* , vec2f(10.f, -37.f), vec2f(8.f, -39.f), vec2f(0.f, -40.f) */, vec2f(-30.f, -30.f), vec2f(-30.f, 30.f)};
    //green
    vec2f def_avg = {0, 0}; 
    //red
    vec2f model_avg = std::reduce(model_rect.begin(), model_rect.end()) / static_cast<float>(model_rect.size());
    //blue
    vec2f area_avg = getCenterOfMass(model_rect);
    //pos, pos + model_avg, pos + area_avg, model
    
    vec2f win_size = {800.f, 800.f};
    AABB big_aabb = AABB::CreateMinMax(vec2f(), win_size); 
    AABB small_aabb = AABB::CreateCenterSize(big_aabb.center(), big_aabb.size() * 0.9f); 
    sys.add(yellow,     "yellow", Parent{sys.world}, sf::Color::Yellow, {big_aabb.bl(), big_aabb.br(), small_aabb.br(), small_aabb.bl()}, true);
    sys.add(magenta,     "magenta", Parent{sys.world}, sf::Color::Magenta, {big_aabb.tl(), big_aabb.tr(), small_aabb.tr(), small_aabb.tl()}, true);
    sys.add(red,     "red", Parent{sys.world}, sf::Color::Red, {big_aabb.bl(), big_aabb.tl(), small_aabb.tl(), small_aabb.bl()}, true);
    sys.add(green,     "green", Parent{sys.world}, sf::Color::Green, {big_aabb.br(), big_aabb.tr(), small_aabb.tr(), small_aabb.br()}, true);
    // sys.add(Entity(), "", Position({100.f, 100.f}), Rotation(0.f), sys.world, sf::Color::White, model_rect);

    auto hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());

    // create the window
    sf::RenderWindow window(sf::VideoMode(win_size.x, win_size.y), "My window", sf::Style::Close);
    // window.setFramerateLimit(60U);
    
    if (!ImGui::SFML::Init(window)) return -1;
    
    ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true;
    sf::Clock delTclock;
    System sys_clone;
    std::vector<vec2f> creation_points;

    // run the program as long as the window is open
    while (window.isOpen()) {
        // check all the window's events that were triggered since the last
        // iteration of the loop
        sf::Event event{};
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed)
                window.close();
            if(event.type == sf::Event::MouseButtonPressed) {
                if(event.mouseButton.button == sf::Mouse::Left) {
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    creation_points.push_back((vec2f)mouse_pos);
                }
            }
            if(event.type == sf::Event::KeyPressed) {
                if(event.key.code == sf::Keyboard::R) {
                    creation_points.clear();
                }
                if(event.key.code == sf::Keyboard::Enter) {
                    sys.add(Entity(), "", sys.world, sf::Color::White, creation_points);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());
                }
                if(event.key.code == sf::Keyboard::V) {
                    auto mouse_pos = sf::Mouse::getPosition(window);
                    Entity t;
                    sys.add(t, "", Position({static_cast<float>(mouse_pos.x), static_cast<float>(mouse_pos.y)}), Rotation(0.f), sys.world, sf::Color::White, model_rect);
                    // sys.rb_sys.get<Rigidbody::lockRotationFlag>(t) = true;
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());
                }
                if(event.key.code == sf::Keyboard::B) {
                    auto m = model_rect;
                    for(auto& p : m) p *= 0.5f;
                    auto mouse_pos = (vec2f)sf::Mouse::getPosition(window);
                    sys.add(Entity(), "", Position(mouse_pos), Rotation(0.f), sys.world, sf::Color::White, m);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());
                }
                if(event.key.code == sf::Keyboard::C) {
                    auto m = model_rect;
                    for(auto& p : m) p *= 1.5f;
                    auto mouse_pos = (vec2f)sf::Mouse::getPosition(window);
                    sys.add(Entity(), "", Position(mouse_pos), Rotation(0.f), sys.world, sf::Color::White, m);
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());
                }
                if(event.key.code == sf::Keyboard::S && !event.key.shift) {
                    sys_clone = sys;
                }
                if(event.key.code == sf::Keyboard::S && event.key.shift) {
                    sys = sys_clone;
                    hierarchy_bfs = Hierarchy::getBFSIndexList(sys.hierarchy.sliceOwner<Parent>());
                }
            }
        }
        auto delTtime = delTclock.restart();
        ImGui::SFML::Update(window, delTtime);

        // clear the window with black color
        
        Transform::updateLocalTransforms(sys.transforms.slice<Position, Rotation, Scale, LocalTransform>());
        
        updateParentTransformByHierarchy(
            sys.transforms.sliceOwner<LocalTransform, GlobalTransform>(),
            sys.hierarchy, hierarchy_bfs);

        
        static ThreadPool tp;
        sys.phy_man.update(sys.transforms, sys.rb_sys, sys.col_sys, sys.mat_sys, /* 1.f / 60.f */ delTtime.asSeconds(), tp);
        window.clear(sf::Color::Black);
        std::vector<sf::Vector2f> positions;
        std::vector<Entity> ids;
        for (auto [e, global_trans] : sys.transforms.sliceOwner<GlobalTransform>()) {
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
        Collider::calcParitionedShapes(sys.col_sys.slice<Collider::ShapeModel, Collider::ShapePartitioned>());
        Collider::updatePartitionedTransformedShapes(
            sys.col_sys.sliceOwner<Collider::ShapePartitioned, Collider::ShapeTransformedPartitioned>(),
            sys.transforms.slice<Transform::GlobalTransform>());
        
        for(auto [shape] : sys.col_sys.slice<Collider::ShapeTransformedPartitioned>()) {
            sf::Vertex vert[2];
            static const sf::Color clr = sf::Color::Green; 
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
        
        {
            auto convertToImVec = [](sf::Vector2f vec) {
                return ImVec2(vec.x, vec.y);
            };
            auto convertToSfVec = [](ImVec2 vec) {
                return sf::Vector2f(vec.x, vec.y);
            };
            auto slider2D = [&](std::string name, sf::Vector2f* vec,
                                sf::Vector2f min, sf::Vector2f max,
                                float scale) {
                ImVec2 tmp = convertToImVec(*vec);
                InputVec2(name.c_str(), &tmp, min, max, scale);
                *vec = tmp;
            };
            ImGui::Begin("settings");
            ImGui::Text("ObjectCount: %zu", sys.transforms.size());
            ImGui::Text("FPS: %f", 1.0 / delTtime.asSeconds());
            static std::string selected = sys.name_table.begin()->second;
            ImGui::Dummy({});
            for (auto [e, name] : sys.name_table) {
                if(name == "")
                    continue;;
                ImGui::SameLine();
                if (ImGui::Button(name.c_str(), {50.f, 20.f})) {
                    selected = name;
                }
            }
            for (auto [e, name] : sys.name_table) {
                if (selected != name) {
                    // if(sys.rb_sys.try_get<Rigidbody::isStaticFlag>(e).has_value())
                    //     *sys.rb_sys.try_get<Rigidbody::isStaticFlag>(e).value() = false;
                    continue;
                }
                *sys.rb_sys.try_get<Rigidbody::Velocity>(e).value() = vec2f();
                ImGui::BeginChild(("tab " + name).c_str());
                ImGui::Text("%s", (name + " settings:").c_str());
                auto& ref_pos =
                    *sys.transforms.try_get<Position>(e).value();
                auto height = window.getView().getSize().y;
                ref_pos.y = height - ref_pos.y;
                slider2D(name + " pos", &ref_pos, {0.f, 0.f},
                         {window.getView().getSize()}, 1.f);
                ref_pos.y = height - ref_pos.y;

                auto& ref_scale =
                    *sys.transforms.try_get<Scale>(e).value();
                slider2D(name + " scale", &ref_scale, {-2.f, -2.f}, {2.f, 2.f},
                         1.f);

                auto& ref_rotate =
                    *sys.transforms.try_get<Rotation>(e).value();
                float& f = ref_rotate;
                ImGui::SliderFloat((name + " rotation").c_str(),
                                   &f, 0.f, M_PI * 2.f);
                ImGui::EndChild();
            }
            ImGui::End();
        }
        ImGui::SFML::Render(window);

        // draw everything here...
        // window.draw(...);

        // end the current frame
        window.display();
    }

    return 0;
}
using namespace ImGui;

bool InputVec2(char const* pLabel, ImVec2* pValue, ImVec2 const vMinValue,
               ImVec2 const vMaxValue, float const fScale /*= 1.0f*/) {
    return SliderScalar2D(pLabel, &pValue->x, &pValue->y, vMinValue.x,
                          vMaxValue.x, vMinValue.y, vMaxValue.y, fScale);
}

ImVec2 operator*(float f, ImVec2 vec) { return {vec.x * f, vec.y * f}; }
ImVec2 operator+(ImVec2 v1, ImVec2 v2) { return {v1.x + v2.x, v1.y + v2.y}; }
ImVec2 operator-(ImVec2 v1, ImVec2 v2) { return {v1.x - v2.x, v1.y - v2.y}; }

bool SliderScalar2D(char const* pLabel, float* fValueX, float* fValueY,
                    const float fMinX, const float fMaxX, const float fMinY,
                    const float fMaxY, float const fZoom /*= 1.0f*/) {
    assert(fMinX < fMaxX);
    assert(fMinY < fMaxY);

    ImGuiID const iID = ImGui::GetID(pLabel);

    ImVec2 const vSizeSubstract =
        1.1f * ImGui::CalcTextSize(std::to_string(1.0f).c_str());

    float const vSizeFull = (GetWindowWidth() - vSizeSubstract.x) * fZoom;
    ImVec2 const vSize(vSizeFull, vSizeFull);

    float const fHeightOffset = ImGui::GetTextLineHeight();
    ImVec2 const vHeightOffset(0.0f, fHeightOffset);

    ImVec2 vPos = GetCursorScreenPos();
    ImRect oRect(vPos + vHeightOffset, vPos + vSize + vHeightOffset);

    ImGui::Text("%s", pLabel);

    ImGui::PushID(iID);

    ImU32 const uFrameCol = ImGui::GetColorU32(ImGuiCol_FrameBg);

    ImVec2 const vOriginPos = ImGui::GetCursorScreenPos();
    ImGui::RenderFrame(oRect.Min, oRect.Max, uFrameCol, false, 0.0f);

    float const fDeltaX = fMaxX - fMinX;
    float const fDeltaY = fMaxY - fMinY;

    bool bModified = false;
    ImVec2 const vSecurity(15.0f, 15.0f);
    if (ImGui::IsMouseHoveringRect(oRect.Min - vSecurity,
                                   oRect.Max + vSecurity) &&
        ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 const vCursorPos = ImGui::GetMousePos() - oRect.Min;

        *fValueX = vCursorPos.x / (oRect.Max.x - oRect.Min.x) * fDeltaX + fMinX;
        *fValueY = fDeltaY -
                   vCursorPos.y / (oRect.Max.y - oRect.Min.y) * fDeltaY + fMinY;

        bModified = true;
    }

    *fValueX = std::min(std::max(*fValueX, fMinX), fMaxX);
    *fValueY = std::min(std::max(*fValueY, fMinY), fMaxY);

    float const fScaleX = (*fValueX - fMinX) / fDeltaX;
    float const fScaleY = 1.0f - (*fValueY - fMinY) / fDeltaY;

    constexpr float fCursorOff = 10.0f;
    float const fXLimit = fCursorOff / oRect.GetWidth();
    float const fYLimit = fCursorOff / oRect.GetHeight();

    ImVec2 const vCursorPos((oRect.Max.x - oRect.Min.x) * fScaleX + oRect.Min.x,
                            (oRect.Max.y - oRect.Min.y) * fScaleY +
                                oRect.Min.y);

    ImDrawList* pDrawList = ImGui::GetWindowDrawList();

    ImVec4 const vBlue(70.0f / 255.0f, 102.0f / 255.0f, 230.0f / 255.0f,
                       1.0f); // TODO: choose from style
    ImVec4 const vOrange(255.0f / 255.0f, 128.0f / 255.0f, 64.0f / 255.0f,
                         1.0f); // TODO: choose from style

    ImS32 const uBlue = ImGui::GetColorU32(vBlue);
    ImS32 const uOrange = ImGui::GetColorU32(vOrange);

    constexpr float fBorderThickness = 2.0f;
    constexpr float fLineThickness = 3.0f;
    constexpr float fHandleRadius = 7.0f;
    constexpr float fHandleOffsetCoef = 2.0f;

    // Cursor
    pDrawList->AddCircleFilled(vCursorPos, 5.0f, uBlue, 16);

    // Vertical Line
    if (fScaleY > 2.0f * fYLimit)
        pDrawList->AddLine(ImVec2(vCursorPos.x, oRect.Min.y + fCursorOff),
                           ImVec2(vCursorPos.x, vCursorPos.y - fCursorOff),
                           uOrange, fLineThickness);
    if (fScaleY < 1.0f - 2.0f * fYLimit)
        pDrawList->AddLine(ImVec2(vCursorPos.x, oRect.Max.y - fCursorOff),
                           ImVec2(vCursorPos.x, vCursorPos.y + fCursorOff),
                           uOrange, fLineThickness);

    // Horizontal Line
    if (fScaleX > 2.0f * fXLimit)
        pDrawList->AddLine(ImVec2(oRect.Min.x + fCursorOff, vCursorPos.y),
                           ImVec2(vCursorPos.x - fCursorOff, vCursorPos.y),
                           uOrange, fLineThickness);
    if (fScaleX < 1.0f - 2.0f * fYLimit)
        pDrawList->AddLine(ImVec2(oRect.Max.x - fCursorOff, vCursorPos.y),
                           ImVec2(vCursorPos.x + fCursorOff, vCursorPos.y),
                           uOrange, fLineThickness);

    char pBufferX[16];
    char pBufferY[16];
    ImFormatString(pBufferX, IM_ARRAYSIZE(pBufferX), "%.5f",
                   *(float const*)fValueX);
    ImFormatString(pBufferY, IM_ARRAYSIZE(pBufferY), "%.5f",
                   *(float const*)fValueY);

    ImU32 const uTextCol =
        ImGui::ColorConvertFloat4ToU32(ImGui::GetStyle().Colors[ImGuiCol_Text]);

    ImVec2 vXSize = ImGui::CalcTextSize(pBufferX);
    ImVec2 vYSize = ImGui::CalcTextSize(pBufferY);
    vYSize.x = 0;

    ImVec2 const vHandlePosX =
        ImVec2(vCursorPos.x, oRect.Max.y + vXSize.x * 0.5f);
    ImVec2 const vHandlePosY = ImVec2(
        oRect.Max.x + fHandleOffsetCoef * fCursorOff + vYSize.x, vCursorPos.y);

    if (ImGui::IsMouseHoveringRect(
            vHandlePosX - ImVec2(fHandleRadius, fHandleRadius) - vSecurity,
            vHandlePosX + ImVec2(fHandleRadius, fHandleRadius) + vSecurity) &&
        ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 const vCursorPos = ImGui::GetMousePos() - oRect.Min;

        *fValueX = vCursorPos.x / (oRect.Max.x - oRect.Min.x) * fDeltaX + fMinX;

        bModified = true;
    } else if (ImGui::IsMouseHoveringRect(
                   vHandlePosY - ImVec2(fHandleRadius, fHandleRadius) -
                       vSecurity,
                   vHandlePosY + ImVec2(fHandleRadius, fHandleRadius) +
                       vSecurity) &&
               ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 const vCursorPos = ImGui::GetMousePos() - oRect.Min;

        *fValueY = fDeltaY -
                   vCursorPos.y / (oRect.Max.y - oRect.Min.y) * fDeltaY + fMinY;

        bModified = true;
    }

    pDrawList->AddText(
        ImVec2(std::min(std::max(vCursorPos.x - vXSize.x * 0.5f, oRect.Min.x),
                        oRect.Min.x + vSize.x - vXSize.x),
               oRect.Max.y + fCursorOff),
        uTextCol, pBufferX);
    pDrawList->AddText(
        ImVec2(oRect.Max.x + fCursorOff,
               std::min(std::max(vCursorPos.y - vYSize.y * 0.5f, oRect.Min.y),
                        oRect.Min.y + vSize.y - vYSize.y)),
        uTextCol, pBufferY);
    ImGui::SetWindowFontScale(1.0f);

    // Borders::Right
    pDrawList->AddCircleFilled(ImVec2(oRect.Max.x, vCursorPos.y), 2.0f, uOrange,
                               3);
    // Handle Right::Y
    pDrawList->AddNgonFilled(
        ImVec2(oRect.Max.x + fHandleOffsetCoef * fCursorOff + vYSize.x,
               vCursorPos.y),
        fHandleRadius, uBlue, 4);
    if (fScaleY > fYLimit)
        pDrawList->AddLine(ImVec2(oRect.Max.x, oRect.Min.y),
                           ImVec2(oRect.Max.x, vCursorPos.y - fCursorOff),
                           uBlue, fBorderThickness);
    if (fScaleY < 1.0f - fYLimit)
        pDrawList->AddLine(ImVec2(oRect.Max.x, oRect.Max.y),
                           ImVec2(oRect.Max.x, vCursorPos.y + fCursorOff),
                           uBlue, fBorderThickness);
    // Borders::Top
    pDrawList->AddCircleFilled(ImVec2(vCursorPos.x, oRect.Min.y), 2.0f, uOrange,
                               3);
    if (fScaleX > fXLimit)
        pDrawList->AddLine(ImVec2(oRect.Min.x, oRect.Min.y),
                           ImVec2(vCursorPos.x - fCursorOff, oRect.Min.y),
                           uBlue, fBorderThickness);
    if (fScaleX < 1.0f - fXLimit)
        pDrawList->AddLine(ImVec2(oRect.Max.x, oRect.Min.y),
                           ImVec2(vCursorPos.x + fCursorOff, oRect.Min.y),
                           uBlue, fBorderThickness);
    // Borders::Left
    pDrawList->AddCircleFilled(ImVec2(oRect.Min.x, vCursorPos.y), 2.0f, uOrange,
                               3);
    if (fScaleY > fYLimit)
        pDrawList->AddLine(ImVec2(oRect.Min.x, oRect.Min.y),
                           ImVec2(oRect.Min.x, vCursorPos.y - fCursorOff),
                           uBlue, fBorderThickness);
    if (fScaleY < 1.0f - fYLimit)
        pDrawList->AddLine(ImVec2(oRect.Min.x, oRect.Max.y),
                           ImVec2(oRect.Min.x, vCursorPos.y + fCursorOff),
                           uBlue, fBorderThickness);
    // Borders::Bottom
    pDrawList->AddCircleFilled(ImVec2(vCursorPos.x, oRect.Max.y), 2.0f, uOrange,
                               3);
    // Handle Bottom::X
    pDrawList->AddNgonFilled(
        ImVec2(vCursorPos.x, oRect.Max.y + vXSize.y * 2.0f), fHandleRadius,
        uBlue, 4);
    if (fScaleX > fXLimit)
        pDrawList->AddLine(ImVec2(oRect.Min.x, oRect.Max.y),
                           ImVec2(vCursorPos.x - fCursorOff, oRect.Max.y),
                           uBlue, fBorderThickness);
    if (fScaleX < 1.0f - fXLimit)
        pDrawList->AddLine(ImVec2(oRect.Max.x, oRect.Max.y),
                           ImVec2(vCursorPos.x + fCursorOff, oRect.Max.y),
                           uBlue, fBorderThickness);

    ImGui::PopID();

    ImGui::Dummy(vHeightOffset);
    ImGui::Dummy(vHeightOffset);
    ImGui::Dummy(vSize);

    char pBufferID[64];
    ImFormatString(pBufferID, IM_ARRAYSIZE(pBufferID), "Values##%d",
                   *(ImS32 const*)&iID);

    if (ImGui::CollapsingHeader(pBufferID)) {
        float const fSpeedX = fDeltaX / 64.0f;
        float const fSpeedY = fDeltaY / 64.0f;

        char pBufferXID[64];
        ImFormatString(pBufferXID, IM_ARRAYSIZE(pBufferXID), "X##%d",
                       *(ImS32 const*)&iID);
        char pBufferYID[64];
        ImFormatString(pBufferYID, IM_ARRAYSIZE(pBufferYID), "Y##%d",
                       *(ImS32 const*)&iID);

        bModified |= ImGui::DragScalar(pBufferXID, ImGuiDataType_Float, fValueX,
                                       fSpeedX, &fMinX, &fMaxX);
        bModified |= ImGui::DragScalar(pBufferYID, ImGuiDataType_Float, fValueY,
                                       fSpeedY, &fMinY, &fMaxY);
    }

    return bModified;
}
