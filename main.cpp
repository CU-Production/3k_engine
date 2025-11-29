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
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <sstream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// ============================================================================
// ECS Framework
// ============================================================================

struct EntityId {
    uint32_t id;
    uint32_t generation;
    
    bool operator==(const EntityId& other) const {
        return id == other.id && generation == other.generation;
    }
    bool operator!=(const EntityId& other) const { return !(*this == other); }
};

namespace std {
    template<> struct hash<EntityId> {
        size_t operator()(const EntityId& e) const {
            return ((size_t)e.id << 32) | e.generation;
        }
    };
}

static const EntityId NULL_ENTITY = {UINT32_MAX, 0};

// Components
struct Transform {
    HMM_Vec2 position;
    float rotation;
    HMM_Vec2 scale;
    EntityId parent;
    
    Transform() : position({0,0}), rotation(0), scale({1,1}), parent(NULL_ENTITY) {}
};

struct Sprite {
    HMM_Vec4 color;
    HMM_Vec2 size;
    sg_image texture;
    
    Sprite() : color({1,1,1,1}), size({100,100}), texture{SG_INVALID_ID} {}
};

struct Rigidbody {
    b2BodyId body;
    
    Rigidbody() : body(b2_nullBodyId) {}
};

struct Script {
    std::string path;
    sol::table instance; // Lua table instance
    bool loaded;
    
    Script() : path(""), loaded(false) {}
};

struct Camera {
    float zoom;
    HMM_Vec2 offset;
    
    Camera() : zoom(1.0f), offset({0,0}) {}
};

// Sparse set component storage
template<typename T>
struct ComponentArray {
    std::vector<EntityId> entities;
    std::vector<T> components;
    std::unordered_map<EntityId, size_t> entity_to_index;
    
    bool has(EntityId e) const {
        return entity_to_index.find(e) != entity_to_index.end();
    }
    
    T* get(EntityId e) {
        auto it = entity_to_index.find(e);
        if (it == entity_to_index.end()) return nullptr;
        return &components[it->second];
    }
    
    T& add(EntityId e, const T& component) {
        if (has(e)) {
            return *get(e);
        }
        size_t index = entities.size();
        entities.push_back(e);
        components.push_back(component);
        entity_to_index[e] = index;
        return components.back();
    }
    
    void remove(EntityId e) {
        auto it = entity_to_index.find(e);
        if (it == entity_to_index.end()) return;
        
        size_t index = it->second;
        size_t last = entities.size() - 1;
        
        if (index != last) {
            entities[index] = entities[last];
            components[index] = components[last];
            entity_to_index[entities[index]] = index;
        }
        
        entities.pop_back();
        components.pop_back();
        entity_to_index.erase(e);
    }
    
    template<typename Fn>
    void each(Fn&& fn) {
        for (size_t i = 0; i < entities.size(); ++i) {
            fn(entities[i], components[i]);
        }
    }
};

// Registry
struct Registry {
    std::vector<uint32_t> free_ids;
    std::vector<uint32_t> generations;
    uint32_t next_id = 0;
    
    ComponentArray<Transform> transforms;
    ComponentArray<Sprite> sprites;
    ComponentArray<Rigidbody> rigidbodies;
    ComponentArray<Script> scripts;
    ComponentArray<Camera> cameras;
    
    EntityId create() {
        EntityId e;
        if (!free_ids.empty()) {
            e.id = free_ids.back();
            free_ids.pop_back();
            e.generation = generations[e.id];
        } else {
            e.id = next_id++;
            e.generation = 0;
            generations.push_back(0);
        }
        return e;
    }
    
    void destroy(EntityId e) {
        if (e.id >= generations.size() || generations[e.id] != e.generation) {
            return; // Invalid entity
        }
        
        // Remove all components
        transforms.remove(e);
        sprites.remove(e);
        rigidbodies.remove(e);
        scripts.remove(e);
        cameras.remove(e);
        
        // Increment generation and add to free list
        generations[e.id]++;
        free_ids.push_back(e.id);
    }
    
    bool valid(EntityId e) const {
        return e.id < generations.size() && generations[e.id] == e.generation;
    }
};

// ============================================================================
// Systems
// ============================================================================

// Physics System: sync Rigidbody <-> Transform
struct PhysicsSystem {
    static void sync_to_physics(Registry& reg, b2WorldId world) {
        reg.rigidbodies.each([&](EntityId e, Rigidbody& rb) {
            if (b2Body_IsValid(rb.body)) {
                Transform* t = reg.transforms.get(e);
                if (t) {
                    b2Body_SetTransform(rb.body, b2Vec2{t->position.X, t->position.Y}, b2MakeRot(t->rotation));
                }
            }
        });
    }
    
    static void sync_from_physics(Registry& reg, b2WorldId world) {
        reg.rigidbodies.each([&](EntityId e, Rigidbody& rb) {
            if (b2Body_IsValid(rb.body)) {
                Transform* t = reg.transforms.get(e);
                if (t) {
                    b2Vec2 pos = b2Body_GetPosition(rb.body);
                    b2Rot rot = b2Body_GetRotation(rb.body);
                    t->position = {pos.x, pos.y};
                    t->rotation = b2Rot_GetAngle(rot);
                }
            }
        });
    }
};

// Scene Serialization
struct SceneSerializer {
    static bool save(const char* path, Registry& reg) {
        std::ofstream file(path);
        if (!file.is_open()) return false;
        
        file << "# Scene File\n";
        
        reg.transforms.each([&](EntityId e, Transform& t) {
            file << "entity " << e.id << " " << e.generation << "\n";
            file << "  transform " << t.position.X << " " << t.position.Y << " " 
                 << t.rotation << " " << t.scale.X << " " << t.scale.Y << "\n";
            
            Sprite* sprite = reg.sprites.get(e);
            if (sprite) {
                file << "  sprite " << sprite->color.X << " " << sprite->color.Y << " " 
                     << sprite->color.Z << " " << sprite->color.W << " "
                     << sprite->size.X << " " << sprite->size.Y << "\n";
            }
            
            Script* script = reg.scripts.get(e);
            if (script && !script->path.empty()) {
                file << "  script " << script->path << "\n";
            }
        });
        
        file.close();
        return true;
    }
    
    static bool load(const char* path, Registry& reg, b2WorldId world) {
        PHYSFS_File* pfile = PHYSFS_openRead(path);
        if (!pfile) return false;
        
        PHYSFS_sint64 filesize = PHYSFS_fileLength(pfile);
        if (filesize <= 0) { PHYSFS_close(pfile); return false; }
        
        std::vector<char> buffer(filesize + 1);
        PHYSFS_readBytes(pfile, buffer.data(), filesize);
        buffer[filesize] = '\0';
        PHYSFS_close(pfile);
        
        std::istringstream iss(buffer.data());
        std::string line;
        EntityId current_entity = NULL_ENTITY;
        
        while (std::getline(iss, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream lss(line);
            std::string cmd;
            lss >> cmd;
            
            if (cmd == "entity") {
                current_entity = reg.create();
            } else if (cmd == "transform" && current_entity != NULL_ENTITY) {
                Transform t;
                lss >> t.position.X >> t.position.Y >> t.rotation >> t.scale.X >> t.scale.Y;
                reg.transforms.add(current_entity, t);
            } else if (cmd == "sprite" && current_entity != NULL_ENTITY) {
                Sprite s;
                lss >> s.color.X >> s.color.Y >> s.color.Z >> s.color.W >> s.size.X >> s.size.Y;
                reg.sprites.add(current_entity, s);
            } else if (cmd == "script" && current_entity != NULL_ENTITY) {
                Script sc;
                lss >> sc.path;
                reg.scripts.add(current_entity, sc);
            }
        }
        
        return true;
    }
};

// Input System
struct InputSystem {
    static bool keys[SAPP_MAX_KEYCODES];
    static bool keys_pressed[SAPP_MAX_KEYCODES];
    static HMM_Vec2 mouse_pos;
    static bool mouse_buttons[3];
    
    static void reset() {
        for (int i = 0; i < SAPP_MAX_KEYCODES; ++i) {
            keys_pressed[i] = false;
        }
    }
    
    static bool get_key(sapp_keycode key) { return keys[key]; }
    static bool get_key_down(sapp_keycode key) { return keys_pressed[key]; }
    static HMM_Vec2 get_mouse_position() { return mouse_pos; }
    static bool get_mouse_button(int button) { return button < 3 ? mouse_buttons[button] : false; }
};

bool InputSystem::keys[SAPP_MAX_KEYCODES] = {};
bool InputSystem::keys_pressed[SAPP_MAX_KEYCODES] = {};
HMM_Vec2 InputSystem::mouse_pos = {0, 0};
bool InputSystem::mouse_buttons[3] = {};

// Script System
struct ScriptSystem {
    static void load_script(Script& script, sol::state* lua) {
        if (script.path.empty() || script.loaded) return;
        
        PHYSFS_File* file = PHYSFS_openRead(script.path.c_str());
        if (!file) return;
        
        PHYSFS_sint64 filesize = PHYSFS_fileLength(file);
        if (filesize <= 0) { PHYSFS_close(file); return; }
        
        std::vector<char> buffer(filesize + 1);
        PHYSFS_readBytes(file, buffer.data(), filesize);
        buffer[filesize] = '\0';
        PHYSFS_close(file);
        
        try {
            sol::load_result loaded_script = lua->load(buffer.data());
            if (loaded_script.valid()) {
                sol::protected_function_result result = loaded_script();
                if (result.valid()) {
                    script.instance = result;
                    script.loaded = true;
                }
            }
        } catch (const std::exception& e) {
            // Script load failed
        }
    }
    
    static void update_scripts(Registry& reg, sol::state* lua, float dt) {
        reg.scripts.each([&](EntityId e, Script& sc) {
            if (!sc.loaded && !sc.path.empty()) {
                load_script(sc, lua);
            }
            
            if (sc.loaded && sc.instance.valid()) {
                sol::optional<sol::function> update_fn = sc.instance["update"];
                if (update_fn) {
                    try {
                        (*update_fn)(dt);
                    } catch (const std::exception& ex) {
                        // Script error
                    }
                }
            }
        });
    }
};

// Asset Manager
struct AssetManager {
    static std::unordered_map<std::string, sg_image> textures;
    
    static sg_image load_texture(const char* path) {
        auto it = textures.find(path);
        if (it != textures.end()) {
            return it->second;
        }
        
        PHYSFS_File* file = PHYSFS_openRead(path);
        if (!file) return sg_image{SG_INVALID_ID};
        
        PHYSFS_sint64 filesize = PHYSFS_fileLength(file);
        if (filesize <= 0) { PHYSFS_close(file); return sg_image{SG_INVALID_ID}; }
        
        std::vector<uint8_t> buffer(filesize);
        PHYSFS_readBytes(file, buffer.data(), filesize);
        PHYSFS_close(file);
        
        int width, height, channels;
        stbi_uc* pixels = stbi_load_from_memory(buffer.data(), (int)filesize, &width, &height, &channels, 4);
        if (!pixels) return sg_image{SG_INVALID_ID};
        
        sg_image_desc img_desc = {};
        img_desc.width = width;
        img_desc.height = height;
        img_desc.pixel_format = SG_PIXELFORMAT_RGBA8;
        img_desc.data.mip_levels[0].ptr = pixels;
        img_desc.data.mip_levels[0].size = (size_t)(width * height * 4);
        
        sg_image img = sg_make_image(&img_desc);
        stbi_image_free(pixels);
        
        textures[path] = img;
        return img;
    }
    
    static void cleanup() {
        for (auto& pair : textures) {
            sg_destroy_image(pair.second);
        }
        textures.clear();
    }
};

std::unordered_map<std::string, sg_image> AssetManager::textures;

static bool show_test_window = true;
static bool show_another_window = false;
static bool show_viewport = true;
static bool show_hierarchy = true;
static bool show_inspector = true;

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
    Registry registry;
    EntityId selected_entity;
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
    
    // Bind input to Lua
    state.lua->set_function("get_key", &InputSystem::get_key);
    state.lua->set_function("get_key_down", &InputSystem::get_key_down);
    state.lua->set_function("get_mouse_pos", &InputSystem::get_mouse_position);
    state.lua->set_function("get_mouse_button", &InputSystem::get_mouse_button);
    
    // Bind utility functions
    state.lua->set_function("log", [](const std::string& msg) {
        printf("[Lua] %s\n", msg.c_str());
    });
    
    // ECS: create sample entities
    state.selected_entity = NULL_ENTITY;
    
    EntityId e1 = state.registry.create();
    state.registry.transforms.add(e1, Transform());
    state.registry.transforms.get(e1)->position = {0, 0};
    state.registry.sprites.add(e1, Sprite());
    state.registry.sprites.get(e1)->color = {1.0f, 0.2f, 0.2f, 1.0f};
    state.registry.sprites.get(e1)->size = {100, 100};
    
    EntityId e2 = state.registry.create();
    state.registry.transforms.add(e2, Transform());
    state.registry.transforms.get(e2)->position = {150, 150};
    state.registry.sprites.add(e2, Sprite());
    state.registry.sprites.get(e2)->color = {0.2f, 1.0f, 0.2f, 1.0f};
    state.registry.sprites.get(e2)->size = {80, 80};

    state.pass_action.colors[0] = { .load_action=SG_LOADACTION_CLEAR, .clear_value={0.0f, 0.0f, 0.0f, 1.0f } };
}

void frame(void) {
    const int width = sapp_width();
    const int height = sapp_height();
    const float dt = (float)sapp_frame_duration();
    
    // Reset per-frame input
    InputSystem::reset();

    // Physics fixed-step
    const float step = 1.0f / 60.0f;
    state.accumulator += dt;
    while (state.accumulator >= step) {
        // Update scripts
        ScriptSystem::update_scripts(state.registry, state.lua, step);
        
        // Sync editor changes to physics
        PhysicsSystem::sync_to_physics(state.registry, state.world);
        
        b2World_Step(state.world, step, 4);
        
        // Sync physics back to transforms
        PhysicsSystem::sync_from_physics(state.registry, state.world);
        
        state.accumulator -= step;
    }

    simgui_new_frame({ width, height, sapp_frame_duration(), sapp_dpi_scale() });
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
                SceneSerializer::save("scene.txt", state.registry);
            }
            if (ImGui::MenuItem("Load Scene", "Ctrl+L")) {
                // Clear current scene
                std::vector<EntityId> to_delete;
                state.registry.transforms.each([&](EntityId e, Transform& t) {
                    to_delete.push_back(e);
                });
                for (auto e : to_delete) {
                    state.registry.destroy(e);
                }
                state.selected_entity = NULL_ENTITY;
                
                SceneSerializer::load("scene.txt", state.registry, state.world);
            }
            ImGui::EndMenu();
        }
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
    if (ImGui::Button("Hierarchy")) show_hierarchy ^= 1;
    if (ImGui::Button("Inspector")) show_inspector ^= 1;
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

    // 4. Hierarchy panel
    if (show_hierarchy) {
        ImGui::SetNextWindowSize(ImVec2(250, 400), ImGuiCond_FirstUseEver);
        ImGui::Begin("Hierarchy", &show_hierarchy);
        
        if (ImGui::Button("Create Entity")) {
            EntityId new_entity = state.registry.create();
            state.registry.transforms.add(new_entity, Transform());
        }
        ImGui::SameLine();
        if (ImGui::Button("Delete Selected") && state.selected_entity != NULL_ENTITY) {
            state.registry.destroy(state.selected_entity);
            state.selected_entity = NULL_ENTITY;
        }
        
        ImGui::Separator();
        
        // List all entities
        state.registry.transforms.each([&](EntityId e, Transform& t) {
            char label[64];
            snprintf(label, sizeof(label), "Entity %u.%u", e.id, e.generation);
            
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
            if (e == state.selected_entity) {
                flags |= ImGuiTreeNodeFlags_Selected;
            }
            
            ImGui::TreeNodeEx(label, flags);
            if (ImGui::IsItemClicked()) {
                state.selected_entity = e;
            }
        });
        
        ImGui::End();
    }
    
    // 5. Inspector panel
    if (show_inspector) {
        ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("Inspector", &show_inspector);
        
        if (state.selected_entity != NULL_ENTITY && state.registry.valid(state.selected_entity)) {
            ImGui::Text("Entity ID: %u.%u", state.selected_entity.id, state.selected_entity.generation);
            ImGui::Separator();
            
            // Transform component
            Transform* transform = state.registry.transforms.get(state.selected_entity);
            if (transform) {
                if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::DragFloat2("Position", &transform->position.X, 1.0f);
                    ImGui::DragFloat("Rotation", &transform->rotation, 0.01f);
                    ImGui::DragFloat2("Scale", &transform->scale.X, 0.01f);
                }
            }
            
            // Sprite component
            Sprite* sprite = state.registry.sprites.get(state.selected_entity);
            if (sprite) {
                if (ImGui::CollapsingHeader("Sprite", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::ColorEdit4("Color", &sprite->color.X);
                    ImGui::DragFloat2("Size", &sprite->size.X, 1.0f, 1.0f, 500.0f);
                    
                    // Texture loading
                    static char tex_path[256] = "";
                    ImGui::InputText("Texture Path", tex_path, sizeof(tex_path));
                    ImGui::SameLine();
                    if (ImGui::Button("Load")) {
                        sg_image img = AssetManager::load_texture(tex_path);
                        if (img.id != SG_INVALID_ID) {
                            sprite->texture = img;
                        }
                    }
                    
                    bool has_tex = sprite->texture.id != SG_INVALID_ID;
                    ImGui::Text("Texture: %s", has_tex ? "Loaded" : "None");
                }
            } else {
                if (ImGui::Button("Add Sprite Component")) {
                    state.registry.sprites.add(state.selected_entity, Sprite());
                }
            }
            
            // Rigidbody component
            Rigidbody* rb = state.registry.rigidbodies.get(state.selected_entity);
            if (rb) {
                if (ImGui::CollapsingHeader("Rigidbody", ImGuiTreeNodeFlags_DefaultOpen)) {
                    bool has_body = b2Body_IsValid(rb->body);
                    ImGui::Text("Box2D Body: %s", has_body ? "Valid" : "None");
                    if (!has_body && ImGui::Button("Create Box2D Body")) {
                        b2BodyDef bodyDef = b2DefaultBodyDef();
                        Transform* t = state.registry.transforms.get(state.selected_entity);
                        if (t) {
                            bodyDef.position = b2Vec2{t->position.X, t->position.Y};
                            bodyDef.rotation = b2MakeRot(t->rotation);
                        }
                        bodyDef.type = b2_dynamicBody;
                        rb->body = b2CreateBody(state.world, &bodyDef);
                        
                        // Add a box shape
                        b2Polygon box = b2MakeBox(50.0f, 50.0f);
                        b2ShapeDef shapeDef = b2DefaultShapeDef();
                        shapeDef.density = 1.0f;
                        b2CreatePolygonShape(rb->body, &shapeDef, &box);
                    }
                }
            } else {
                if (ImGui::Button("Add Rigidbody Component")) {
                    state.registry.rigidbodies.add(state.selected_entity, Rigidbody());
                }
            }
            
            // Script component
            Script* script = state.registry.scripts.get(state.selected_entity);
            if (script) {
                if (ImGui::CollapsingHeader("Script", ImGuiTreeNodeFlags_DefaultOpen)) {
                    char buf[256];
                    strncpy(buf, script->path.c_str(), sizeof(buf));
                    buf[sizeof(buf)-1] = '\0';
                    if (ImGui::InputText("Script Path", buf, sizeof(buf))) {
                        script->path = buf;
                    }
                }
            } else {
                if (ImGui::Button("Add Script Component")) {
                    state.registry.scripts.add(state.selected_entity, Script());
                }
            }
            
            // Camera component
            Camera* camera = state.registry.cameras.get(state.selected_entity);
            if (camera) {
                if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::DragFloat("Zoom", &camera->zoom, 0.01f, 0.1f, 10.0f);
                    ImGui::DragFloat2("Offset", &camera->offset.X, 1.0f);
                }
            } else {
                if (ImGui::Button("Add Camera Component")) {
                    state.registry.cameras.add(state.selected_entity, Camera());
                }
            }
            
        } else {
            ImGui::TextDisabled("No entity selected");
        }
        
        ImGui::End();
    }
    
    // 6. Viewport window for 2D drawing
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
        
        // Optional: add grid
        const float GRID_STEP = 64.0f;
        for (float x = fmodf(canvas_p0.x, GRID_STEP); x < canvas_p1.x; x += GRID_STEP)
            dl->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
        for (float y = fmodf(canvas_p0.y, GRID_STEP); y < canvas_p1.y; y += GRID_STEP)
            dl->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), IM_COL32(200, 200, 200, 40));
        
        // Render all entities with Transform + Sprite
        ImVec2 viewport_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);
        state.registry.transforms.each([&](EntityId e, Transform& t) {
            Sprite* sprite = state.registry.sprites.get(e);
            if (sprite) {
                ImVec2 world_pos = ImVec2(viewport_center.x + t.position.X, viewport_center.y - t.position.Y);
                
                // Apply rotation and scale
                float cos_r = cosf(t.rotation);
                float sin_r = sinf(t.rotation);
                ImVec2 scaled_size = ImVec2(sprite->size.X * t.scale.X, sprite->size.Y * t.scale.Y);
                
                // Calculate rotated quad corners
                ImVec2 corners[4];
                float hw = scaled_size.x * 0.5f;
                float hh = scaled_size.y * 0.5f;
                
                // Local corners before rotation
                ImVec2 local[4] = {
                    {-hw, -hh}, // top-left
                    { hw, -hh}, // top-right
                    { hw,  hh}, // bottom-right
                    {-hw,  hh}  // bottom-left
                };
                
                // Rotate and translate to world position
                for (int i = 0; i < 4; ++i) {
                    float rx = local[i].x * cos_r - local[i].y * sin_r;
                    float ry = local[i].x * sin_r + local[i].y * cos_r;
                    corners[i] = ImVec2(world_pos.x + rx, world_pos.y + ry);
                }
                
                ImU32 col = IM_COL32((int)(sprite->color.X*255), (int)(sprite->color.Y*255), 
                                     (int)(sprite->color.Z*255), (int)(sprite->color.W*255));
                
                // Draw filled quad
                dl->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], col);
                
                // Highlight selected
                if (e == state.selected_entity) {
                    dl->AddQuad(corners[0], corners[1], corners[2], corners[3], IM_COL32(255, 255, 0, 255), 2.0f);
                }
            }
        });
        
        // Handle viewport click to select entity
        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            EntityId clicked = NULL_ENTITY;
            
            state.registry.transforms.each([&](EntityId e, Transform& t) {
                Sprite* sprite = state.registry.sprites.get(e);
                if (sprite) {
                    ImVec2 world_pos = ImVec2(viewport_center.x + t.position.X, viewport_center.y - t.position.Y);
                    ImVec2 half_size = ImVec2(sprite->size.X * t.scale.X * 0.5f, sprite->size.Y * t.scale.Y * 0.5f);
                    
                    // Simple AABB test (doesn't account for rotation, but good enough for selection)
                    if (mouse_pos.x >= world_pos.x - half_size.x && mouse_pos.x <= world_pos.x + half_size.x &&
                        mouse_pos.y >= world_pos.y - half_size.y && mouse_pos.y <= world_pos.y + half_size.y) {
                        clicked = e;
                    }
                }
            });
            
            state.selected_entity = (clicked != NULL_ENTITY) ? clicked : NULL_ENTITY;
        }
        
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
    AssetManager::cleanup();
    sgimgui_discard(&state.sgimgui);
    simgui_shutdown();
    b2DestroyWorld(state.world);
    if (state.lua) { delete state.lua; state.lua = nullptr; }
    PHYSFS_deinit();
    sg_shutdown();
}

void event(const sapp_event* e) {
    simgui_handle_event(e);
    
    // Update input system
    if (e->type == SAPP_EVENTTYPE_KEY_DOWN) {
        InputSystem::keys[e->key_code] = true;
        InputSystem::keys_pressed[e->key_code] = true;
    } else if (e->type == SAPP_EVENTTYPE_KEY_UP) {
        InputSystem::keys[e->key_code] = false;
    } else if (e->type == SAPP_EVENTTYPE_MOUSE_MOVE) {
        InputSystem::mouse_pos = {e->mouse_x, e->mouse_y};
    } else if (e->type == SAPP_EVENTTYPE_MOUSE_DOWN) {
        if (e->mouse_button < 3) {
            InputSystem::mouse_buttons[e->mouse_button] = true;
        }
    } else if (e->type == SAPP_EVENTTYPE_MOUSE_UP) {
        if (e->mouse_button < 3) {
            InputSystem::mouse_buttons[e->mouse_button] = false;
        }
    }
}

sapp_desc sokol_main(int argc, char* argv[]) {
    sapp_desc _sapp_desc{};
    _sapp_desc.init_cb = init;
    _sapp_desc.frame_cb = frame;
    _sapp_desc.cleanup_cb = cleanup;
    _sapp_desc.event_cb = event;
    _sapp_desc.width = 1280;
    _sapp_desc.height = 720;
    _sapp_desc.window_title = "sokol-app";
    _sapp_desc.icon.sokol_default = true;
    _sapp_desc.logger.func = slog_func;
    return _sapp_desc;
}
