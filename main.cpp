#define SOKOL_IMPL
#define SOKOL_GLCORE
#define SOKOL_TRACE_HOOKS
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_glue.h"
// removed: #include "sokol_gp.h"
#include "HandmadeMath.h"
#include <vector>

#define SOKOL_IMGUI_IMPL
#include "imgui.h"
#include "util/sokol_imgui.h"
#include "util/sokol_gfx_imgui.h"
#include "box2d/box2d.h"
#include "physfs.h"
#include "sol/sol.hpp"

static bool show_test_window = true;
static bool show_another_window = false;
static bool show_viewport = true;

struct Vertex {
    HMM_Vec4 pos;
    HMM_Vec4 color;
};

// application state
static struct {
    sg_pass_action pass_action;
    sgimgui_t sgimgui;
    b2WorldId world;
    float accumulator;
    sol::state* lua;
} state;

void init(void) {
    sg_desc _sg_desc{};
    _sg_desc.environment = sglue_environment();
    _sg_desc.logger.func = slog_func;
    sg_setup(_sg_desc);


    simgui_desc_t _simgui_desc{};
    _simgui_desc.logger.func = slog_func;
    simgui_setup(&_simgui_desc);

    sgimgui_desc_t _sgimgui_desc{};
    sgimgui_init(&state.sgimgui, &_sgimgui_desc);

    // Box2D world
    b2WorldDef wdef = b2DefaultWorldDef();
    wdef.gravity = B2_LITERAL(b2Vec2){0.0f, -10.0f};
    state.world = b2CreateWorld(&wdef);
    state.accumulator = 0.0f;

    // PhysFS: init and mount current directory
    PHYSFS_init(nullptr);
    PHYSFS_mount(".", nullptr, 1);

    // Lua (sol2)
    state.lua = new sol::state();
    state.lua->open_libraries(sol::lib::base, sol::lib::math);

    state.pass_action.colors[0] = { .load_action=SG_LOADACTION_CLEAR, .clear_value={0.0f, 0.0f, 0.0f, 1.0f } };
}

void frame(void) {
    const int width = sapp_width();
    const int height = sapp_height();

    // Physics fixed-step
    const float dt = (float)sapp_frame_duration();
    const float step = 1.0f / 60.0f;
    state.accumulator += dt;
    while (state.accumulator >= step) {
        b2World_Step(state.world, step, 4);
        state.accumulator -= step;
    }

    simgui_new_frame({ width, height, sapp_frame_duration(), sapp_dpi_scale() });
    if (ImGui::BeginMainMenuBar()) {
        sgimgui_draw_menu(&state.sgimgui, "sokol-gfx");
        ImGui::EndMainMenuBar();
    }

    // 1. Show a simple window
    static float f = 0.0f;
    ImGui::Text("Hello, world!");
    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
    ImGui::ColorEdit3("clear color", &state.pass_action.colors[0].clear_value.r);
    if (ImGui::Button("Test Window")) show_test_window ^= 1;
    if (ImGui::Button("Another Window")) show_another_window ^= 1;
    if (ImGui::Button("Viewport")) show_viewport ^= 1;
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Text("w: %d, h: %d, dpi_scale: %.1f", sapp_width(), sapp_height(), sapp_dpi_scale());
    if (ImGui::Button(sapp_is_fullscreen() ? "Switch to windowed" : "Switch to fullscreen")) {
        sapp_toggle_fullscreen();
    }

    // 2. Show another simple window
    if (show_another_window) {
        ImGui::SetNextWindowSize(ImVec2(200,100), ImGuiCond_FirstUseEver);
        ImGui::Begin("Another Window", &show_another_window);
        ImGui::Text("Hello");
        ImGui::End();
    }

    // 3. ImGui demo window
    if (show_test_window) {
        ImGui::SetNextWindowPos(ImVec2(460, 20), ImGuiCond_FirstUseEver);
        ImGui::ShowDemoWindow();
    }

    // 4. Viewport window for 2D drawing
    if (show_viewport) {
        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
        ImGui::Begin("Viewport", &show_viewport);
        
        // Get viewport draw list and window bounds
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
        
        // Draw background
        dl->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
        dl->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));
        
        // Draw sample rectangle in viewport
        ImVec2 center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);
        ImVec2 p0 = ImVec2(center.x - 50.0f, center.y - 50.0f);
        ImVec2 p1 = ImVec2(center.x + 50.0f, center.y + 50.0f);
        dl->AddRectFilled(p0, p1, IM_COL32(255, 50, 50, 255));
        
        // Optional: add grid
        const float GRID_STEP = 64.0f;
        for (float x = fmodf(canvas_p0.x, GRID_STEP); x < canvas_p1.x; x += GRID_STEP)
            dl->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
        for (float y = fmodf(canvas_p0.y, GRID_STEP); y < canvas_p1.y; y += GRID_STEP)
            dl->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), IM_COL32(200, 200, 200, 40));
        
        ImGui::End();
    }

    sg_pass _sg_pass{};
    _sg_pass = { .action = state.pass_action, .swapchain = sglue_swapchain() };

    sg_begin_pass(&_sg_pass);
    // removed: sgp_flush / sgp_end
    sgimgui_draw(&state.sgimgui);
    simgui_render();
    sg_end_pass();
    sg_commit();
}

void cleanup(void) {
    sgimgui_discard(&state.sgimgui);
    simgui_shutdown();
    b2DestroyWorld(state.world);
    if (state.lua) { delete state.lua; state.lua = nullptr; }
    PHYSFS_deinit();
    sg_shutdown();
}

void input(const sapp_event* ev) {
    simgui_handle_event(ev);
}

sapp_desc sokol_main(int argc, char* argv[]) {
    sapp_desc _sapp_desc{};
    _sapp_desc.init_cb = init;
    _sapp_desc.frame_cb = frame;
    _sapp_desc.cleanup_cb = cleanup;
    _sapp_desc.event_cb = input;
    _sapp_desc.width = 1280;
    _sapp_desc.height = 720;
    _sapp_desc.window_title = "sokol-app";
    _sapp_desc.icon.sokol_default = true;
    _sapp_desc.logger.func = slog_func;
    return _sapp_desc;
}
