#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>
#include <unordered_set>

#include "core/group.hpp"
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

struct Position : public sf::Vector2f {};
struct Rotation {
    float val = 0.f;
};
struct Scale : public sf::Vector2f {};
struct LocalTransform : public sf::Transform {};
struct GlobalTransform : public sf::Transform {};

struct Parent : public Entity {};
struct Children : public std::vector<Entity> {};

typedef Group<Position, Rotation, Scale, LocalTransform, GlobalTransform>
    TransformGroup;
typedef Group<Parent, Children> HierarchyGroup;

struct LayerInfo {
    std::vector<size_t> path;
    int max_layer = -1;
};
LayerInfo getLayerValues(Slice<Entity, Parent> slice) {
    LayerInfo result;
    std::map<Entity, int> layer_values;
    std::vector<size_t> last_layer_member_index;
    std::vector<size_t> first_layer_member_index;
    result.path = std::vector<size_t>(slice.size(), -1);
    size_t idx = 0;
    for (auto [id, parent] : slice) {
        int my_layer;
        if (id == parent) {
            my_layer = 0;
        } else {
            my_layer = layer_values.at(parent) + 1;
        }
        layer_values.insert_or_assign(id, my_layer);
        if (my_layer > result.max_layer) {
            result.max_layer = my_layer;
            last_layer_member_index.push_back(idx);
            first_layer_member_index.push_back(idx);
        } else {
            result.path[last_layer_member_index[my_layer]] = idx;
            last_layer_member_index[my_layer] = idx;
        }
        idx++;
    }
    for (int layer = 0; layer < result.max_layer; layer++) {
        if (result.path[last_layer_member_index[layer]] == (size_t)-1) {
            result.path[last_layer_member_index[layer]] =
                first_layer_member_index[layer + 1];
        }
    }
    for (auto p : result.path) {
        std::cerr << p << "\t";
    }
    std::cerr << "\n";
    return result;
}
void updateLocalTransforms(
    Slice<Entity, Position, Rotation, Scale, LocalTransform> slice) {
    for (auto [e, pos, rot, scale, trans] : slice) {
        trans = {sf::Transform::Identity};
        trans.translate(pos);
        trans.rotate(rot.val);
        trans.scale(scale);
    }
}
void updateParentTransformByHierarchy(
    Slice<Entity, LocalTransform, GlobalTransform> slice,
    const HierarchyGroup& hierarchy, LayerInfo layer_info) {
    size_t to_update = 0;
    int iter = 0;
    while (to_update < slice.size()) {
        auto [e, my_trans, global_trans] = slice[to_update];
        to_update = layer_info.path[to_update];

        auto parent_maybe = hierarchy.getComponent<Parent>(e);
        assert(parent_maybe.has_value());
        auto parent = *parent_maybe.value();

        if (parent == e)
            continue;

        auto global_trans_maybe = slice.getComponent<GlobalTransform>(parent);
        assert(global_trans_maybe.has_value());
        global_trans = *global_trans_maybe.value();
        global_trans.combine(my_trans);
    }
}
struct System {
    TransformGroup transforms;
    HierarchyGroup hierarchy;
    Entity world;
    std::unordered_map<Entity, sf::Color> color_table;
    std::unordered_map<Entity, std::string> name_table;
    void add(Entity id, std::string name, Position p, Rotation r, Parent parent,
             sf::Color color) {
        name_table[id] = name;
        transforms.push_back(id, p, r);
        hierarchy.push_back(id, parent, Children{});
        hierarchy.getComponent<Children>(parent).value()->push_back(id);
        color_table[id] = color;
    }
    System() {
        std::get<Parent>(hierarchy.default_values) = {world};
        std::get<LocalTransform>(transforms.default_values) =
            LocalTransform{sf::Transform::Identity};
        std::get<Scale>(transforms.default_values) = {{1.f, 1.f}};
        std::get<GlobalTransform>(transforms.default_values) =
            GlobalTransform{sf::Transform::Identity};
        hierarchy.push_back(world);
        transforms.push_back(world, Position{}, Rotation{0.f});
    }
};

int main() {

    System sys;


    Entity red;
    Entity magenta;
    Entity yellow;
    Entity green;
    Entity blue;
    sys.add(red,     "red",     Position{{100.f, 100.f}}, Rotation{45.f}, Parent{sys.world}, sf::Color::Red);
    sys.add(magenta, "magenta", Position{{100.f, 100.f}}, Rotation{},     Parent{red},       sf::Color::Magenta);
    sys.add(yellow,  "yellow",  Position{{10.f,  0.f}},   Rotation{},     Parent{magenta},   sf::Color::Yellow);
    sys.add(green,   "green",   Position{{400.f, 100.f}}, Rotation{},     Parent{sys.world}, sf::Color::Green);
    sys.add(blue,    "blue",    Position{{600.f, 100.f}}, Rotation{},     Parent{sys.world}, sf::Color::Blue);

    auto info = getLayerValues(sys.hierarchy.slice<Parent>());
    std::cerr << info.max_layer << "\n";

    // create the window
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "My window");
    assert(ImGui::SFML::Init(window));
    ImGui::GetIO().ConfigWindowsMoveFromTitleBarOnly = true;
    sf::Clock delTclock;

    // run the program as long as the window is open
    while (window.isOpen()) {
        // check all the window's events that were triggered since the last
        // iteration of the loop
        sf::Event event;
        while (window.pollEvent(event)) {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed)
                window.close();
            ImGui::SFML::ProcessEvent(event);
        }
        ImGui::SFML::Update(window, delTclock.restart());

        // clear the window with black color
        window.clear(sf::Color::Black);
        updateLocalTransforms(
            sys.transforms.slice<Position, Rotation, Scale, LocalTransform>());
        updateParentTransformByHierarchy(
            sys.transforms.slice<LocalTransform, GlobalTransform>(),
            sys.hierarchy, info);

        std::vector<sf::Vector2f> positions;
        std::vector<Entity> ids;
        for (auto [e, global_trans] : sys.transforms.slice<GlobalTransform>()) {
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
            static std::string selected;
            ImGui::Dummy({});
            for (auto [e, name] : sys.name_table) {
                ImGui::SameLine();
                if (ImGui::Button(name.c_str(), {50.f, 20.f})) {
                    selected = name;
                }
            }
            for (auto [e, name] : sys.name_table) {
                if (selected != name) {
                    continue;
                }
                ImGui::BeginChild(("tab " + name).c_str());
                ImGui::Text("%s", (name + " settings:").c_str());
                auto& ref_pos =
                    *sys.transforms.getComponent<Position>(e).value();
                auto height = window.getView().getSize().y;
                ref_pos.y = height - ref_pos.y;
                slider2D(name + " pos", &ref_pos, {0.f, 0.f},
                         {window.getView().getSize()}, 1.f);
                ref_pos.y = height - ref_pos.y;

                auto& ref_scale =
                    *sys.transforms.getComponent<Scale>(e).value();
                slider2D(name + " scale", &ref_scale, {-2.f, -2.f}, {2.f, 2.f},
                         1.f);

                auto& ref_rotate =
                    *sys.transforms.getComponent<Rotation>(e).value();
                ImGui::SliderFloat((name + " rotation").c_str(),
                                   &ref_rotate.val, 0.f, 360.f);
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

    ImGui::SetWindowFontScale(0.75f);

    ImVec2 const vXSize = ImGui::CalcTextSize(pBufferX);
    ImVec2 const vYSize = ImGui::CalcTextSize(pBufferY);

    ImVec2 const vHandlePosX =
        ImVec2(vCursorPos.x, oRect.Max.y + vYSize.x * 0.5f);
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
