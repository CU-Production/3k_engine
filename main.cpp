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

#include "nfd.h"

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
    b2BodyType body_type;
    bool fixed_rotation;
    float density;
    float friction;
    float restitution;
    
    Rigidbody() : body(b2_nullBodyId), body_type(b2_dynamicBody), 
                  fixed_rotation(false), density(1.0f), friction(0.3f), restitution(0.0f) {}
};

struct Script {
    std::string path;
    sol::table instance; // Lua table instance
    sol::environment env; // Script environment with entity_id
    bool loaded;
    EntityId entity; // Reference to owner entity
    
    Script() : path(""), loaded(false), entity(NULL_ENTITY) {}
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
            
            Rigidbody* rb = reg.rigidbodies.get(e);
            if (rb) {
                file << "  rigidbody " << (int)rb->body_type << " " 
                     << (rb->fixed_rotation ? 1 : 0) << " "
                     << rb->density << " " << rb->friction << " " << rb->restitution << "\n";
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
        std::string content;
        
        // Try PhysFS first
        PHYSFS_File* pfile = PHYSFS_openRead(path);
        if (pfile) {
            PHYSFS_sint64 filesize = PHYSFS_fileLength(pfile);
            if (filesize > 0) {
                std::vector<char> buffer(filesize + 1);
                PHYSFS_readBytes(pfile, buffer.data(), filesize);
                buffer[filesize] = '\0';
                content = buffer.data();
            }
            PHYSFS_close(pfile);
        } else {
            // Fallback to std::ifstream for absolute paths
            std::ifstream file(path);
            if (!file.is_open()) return false;
            
            std::stringstream buffer;
            buffer << file.rdbuf();
            content = buffer.str();
            file.close();
        }
        
        if (content.empty()) return false;
        
        std::istringstream iss(content);
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
            } else if (cmd == "rigidbody" && current_entity != NULL_ENTITY) {
                Rigidbody rb;
                int body_type_int, fixed_rot_int;
                lss >> body_type_int >> fixed_rot_int >> rb.density >> rb.friction >> rb.restitution;
                rb.body_type = (b2BodyType)body_type_int;
                rb.fixed_rotation = (fixed_rot_int != 0);
                
                // Create Box2D body
                b2BodyDef bodyDef = b2DefaultBodyDef();
                Transform* t = reg.transforms.get(current_entity);
                if (t) {
                    bodyDef.position = b2Vec2{t->position.X, t->position.Y};
                    bodyDef.rotation = b2MakeRot(t->rotation);
                }
                bodyDef.type = rb.body_type;
                bodyDef.fixedRotation = rb.fixed_rotation;
                rb.body = b2CreateBody(world, &bodyDef);
                
                // Add a box shape
                Sprite* sprite = reg.sprites.get(current_entity);
                float hw = sprite ? sprite->size.X * 0.5f : 50.0f;
                float hh = sprite ? sprite->size.Y * 0.5f : 50.0f;
                b2Polygon box = b2MakeBox(hw, hh);
                b2ShapeDef shapeDef = b2DefaultShapeDef();
                shapeDef.density = rb.density;
                shapeDef.material.friction = rb.friction;
                shapeDef.material.restitution = rb.restitution;
                b2CreatePolygonShape(rb.body, &shapeDef, &box);
                
                reg.rigidbodies.add(current_entity, rb);
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
    static void load_script(Script& script, sol::state* lua, EntityId e, Registry& reg) {
        if (script.path.empty() || script.loaded) return;
        script.entity = e;
        
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
                // Create environment for this script with entity_id and generation
                script.env = sol::environment(*lua, sol::create, lua->globals());
                script.env["entity_id"] = e.id;
                script.env["entity_generation"] = e.generation;
                
                // Execute script in its own environment
                sol::protected_function_result result = loaded_script(script.env);
                if (result.valid()) {
                    script.instance = result;
                    script.loaded = true;
                    
                    // Set environment for all script functions
                    sol::optional<sol::function> init_fn = script.instance["init"];
                    if (init_fn) {
                        sol::set_environment(script.env, *init_fn);
                        (*init_fn)();
                    }
                    
                    // Set environment for update function
                    sol::optional<sol::function> update_fn = script.instance["update"];
                    if (update_fn) {
                        sol::set_environment(script.env, *update_fn);
                    }
                }
            }
        } catch (const std::exception& e) {
            // Script load failed
        }
    }
    
    static void update_scripts(Registry& reg, sol::state* lua, float dt) {
        reg.scripts.each([&](EntityId e, Script& sc) {
            if (!sc.loaded && !sc.path.empty()) {
                load_script(sc, lua, e, reg);
            }
            
            if (sc.loaded && sc.instance.valid()) {
                // Update entity_generation in environment before calling update
                sc.env["entity_generation"] = e.generation;
                
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

static bool show_test_window = false;
static bool show_another_window = false;
static bool show_viewport = true;
static bool show_hierarchy = true;
static bool show_inspector = true;
static bool show_console = true;
static bool show_assets = false;

// Console log buffer
static std::vector<std::string> console_logs;
static void log_console(const std::string& msg) {
    console_logs.push_back(msg);
    if (console_logs.size() > 1000) {
        console_logs.erase(console_logs.begin());
    }
}

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
    bool play_mode; // Editor vs Play mode
    std::string current_scene_path;
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
    
    // Enable docking
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    
    // Apply professional dark theme
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.TabRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.TabBorderSize = 1.0f;
    
    // Professional dark color scheme
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.13f, 0.13f, 0.13f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    colors[ImGuiCol_Border] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.18f, 0.18f, 0.18f, 1.00f);
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.55f, 0.55f, 0.55f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.68f, 1.00f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_DockingPreview] = ImVec4(0.26f, 0.59f, 0.98f, 0.70f);
    colors[ImGuiCol_DragDropTarget] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_NavHighlight] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);
    
    style.ItemSpacing = ImVec2(8.0f, 4.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
    style.IndentSpacing = 20.0f;
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 10.0f;

    // Box2D world
    b2WorldDef wdef = b2DefaultWorldDef();
    wdef.gravity = B2_LITERAL(b2Vec2){0.0f, -800.0f};
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
    
    // Bind component access to Lua
    state.lua->set_function("get_transform", [](uint32_t entity_id, uint32_t generation) -> sol::optional<sol::table> {
        EntityId e = {entity_id, generation};
        Transform* t = state.registry.transforms.get(e);
        if (!t) return sol::nullopt;
        
        sol::table result = state.lua->create_table();
        result["x"] = t->position.X;
        result["y"] = t->position.Y;
        result["rotation"] = t->rotation;
        return result;
    });
    
    state.lua->set_function("set_transform", [](uint32_t entity_id, uint32_t generation, float x, float y) {
        EntityId e = {entity_id, generation};
        Transform* t = state.registry.transforms.get(e);
        if (t) {
            t->position.X = x;
            t->position.Y = y;
        }
    });

    state.lua->set_function("get_velocity", [](uint32_t entity_id, uint32_t generation) -> sol::optional<sol::table> {
        EntityId e = {entity_id, generation};
        Rigidbody* rb = state.registry.rigidbodies.get(e);
        if (!rb || !b2Body_IsValid(rb->body)) return sol::nullopt;
        
        b2Vec2 vel = b2Body_GetLinearVelocity(rb->body);
        sol::table result = state.lua->create_table();
        result["x"] = vel.x;
        result["y"] = vel.y;
        return result;
    });
    
    state.lua->set_function("set_velocity", [](uint32_t entity_id, uint32_t generation, float vx, float vy) {
        EntityId e = {entity_id, generation};
        Rigidbody* rb = state.registry.rigidbodies.get(e);
        if (rb && b2Body_IsValid(rb->body)) {
            b2Body_SetLinearVelocity(rb->body, b2Vec2{vx, vy});
        }
    });
    
    state.lua->set_function("apply_impulse", [](uint32_t entity_id, uint32_t generation, float ix, float iy) {
        EntityId e = {entity_id, generation};
        Rigidbody* rb = state.registry.rigidbodies.get(e);
        if (rb && b2Body_IsValid(rb->body)) {
            b2Body_ApplyLinearImpulseToCenter(rb->body, b2Vec2{ix, iy}, true);
        }
    });
    
    state.lua->set_function("destroy_entity", [](uint32_t entity_id, uint32_t generation) {
        EntityId e = {entity_id, generation};
        if (state.registry.valid(e)) {
            state.registry.destroy(e);
            log_console("Entity " + std::to_string(entity_id) + " destroyed by script");
        }
    });
    
    // Bind utility functions
    state.lua->set_function("log", [](const std::string& msg) {
        log_console("[Lua] " + msg);
    });
    
    // Global game state for scripts
    state.lua->set("game_over", false);
    state.lua->set("game_score", 0);
    
    log_console("Engine initialized");
    
    // ECS: create sample entities
    state.selected_entity = NULL_ENTITY;
    state.play_mode = false;
    state.current_scene_path = "scene.txt";
    
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

    // Initialize NFD
    NFD_Init();

    state.pass_action.colors[0].load_action = SG_LOADACTION_CLEAR;
    state.pass_action.colors[0].clear_value = {0.0f, 0.0f, 0.0f, 1.0f};
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
        // Only update scripts and physics in play mode
        if (state.play_mode) {
            // Update scripts
            ScriptSystem::update_scripts(state.registry, state.lua, step);
            
            // Sync editor changes to physics
            PhysicsSystem::sync_to_physics(state.registry, state.world);
            
            b2World_Step(state.world, step, 4);
            
            // Sync physics back to transforms
            PhysicsSystem::sync_from_physics(state.registry, state.world);
        }
        
        state.accumulator -= step;
    }

    simgui_new_frame({ width, height, sapp_frame_duration(), sapp_dpi_scale() });
    
    // Setup dockspace
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar();
    ImGui::PopStyleVar(2);
    
    // DockSpace
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);
    // Menu Bar
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
                nfdchar_t* outPath = nullptr;
                nfdfilteritem_t filters[1] = { { "Scene", "txt" } };
                nfdresult_t result = NFD_SaveDialog(&outPath, filters, 1, nullptr, "scene.txt");
                if (result == NFD_OKAY) {
                    SceneSerializer::save(outPath, state.registry);
                    state.current_scene_path = outPath;
                    NFD_FreePath(outPath);
                    log_console("Scene saved: " + std::string(outPath));
                }
            }
            if (ImGui::MenuItem("Load Scene", "Ctrl+L")) {
                nfdchar_t* outPath = nullptr;
                nfdfilteritem_t filters[1] = { { "Scene", "txt" } };
                nfdresult_t result = NFD_OpenDialog(&outPath, filters, 1, nullptr);
                if (result == NFD_OKAY) {
                    // Clear current scene
                    std::vector<EntityId> to_delete;
                    state.registry.transforms.each([&](EntityId e, Transform& t) {
                        to_delete.push_back(e);
                    });
                    for (auto e : to_delete) {
                        state.registry.destroy(e);
                    }
                    state.selected_entity = NULL_ENTITY;
                    
                    SceneSerializer::load(outPath, state.registry, state.world);
                    state.current_scene_path = outPath;
                    NFD_FreePath(outPath);
                    log_console("Scene loaded: " + std::string(outPath));
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                // Could add exit logic here
            }
            ImGui::EndMenu();
        }
        
        // Play/Stop mode toggle
        if (ImGui::BeginMenu("Scene")) {
            if (state.play_mode) {
                if (ImGui::MenuItem("Stop", "F5")) {
                    state.play_mode = false;
                    // Reload scene to reset state
                    std::vector<EntityId> to_delete;
                    state.registry.transforms.each([&](EntityId e, Transform& t) {
                        to_delete.push_back(e);
                    });
                    for (auto e : to_delete) {
                        state.registry.destroy(e);
                    }
                    state.selected_entity = NULL_ENTITY;
                    SceneSerializer::load(state.current_scene_path.c_str(), state.registry, state.world);
                    log_console("Stopped play mode");
                }
            } else {
                if (ImGui::MenuItem("Play", "F5")) {
                    // Save current state before playing
                    SceneSerializer::save("_temp_editor_state.txt", state.registry);
                    state.play_mode = true;
                    log_console("Started play mode");
                }
            }
            ImGui::EndMenu();
        }
        
        // Windows menu
        if (ImGui::BeginMenu("Window")) {
            ImGui::MenuItem("Hierarchy", nullptr, &show_hierarchy);
            ImGui::MenuItem("Inspector", nullptr, &show_inspector);
            ImGui::MenuItem("Viewport", nullptr, &show_viewport);
            ImGui::MenuItem("Console", nullptr, &show_console);
            ImGui::MenuItem("Assets", nullptr, &show_assets);
            ImGui::Separator();
            ImGui::MenuItem("Demo Window", nullptr, &show_test_window);
            ImGui::EndMenu();
        }
        
        sgimgui_draw_menu(&state.sgimgui, "Graphics");
        
        // Status indicator with better styling
        ImGui::Separator();
        float status_width = 120.0f;
        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - status_width - 20.0f);
        
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 2.0f));
        if (state.play_mode) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 0.3f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.6f, 0.2f, 0.5f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
            ImGui::Button("  PLAYING  ");
            ImGui::PopStyleColor(3);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.5f, 0.2f, 0.3f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.6f, 0.5f, 0.2f, 0.5f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.9f, 0.4f, 1.0f));
            ImGui::Button("  EDITING  ");
            ImGui::PopStyleColor(3);
        }
        ImGui::PopStyleVar();
        
        ImGui::EndMenuBar();
    }
    
    ImGui::End(); // DockSpace
    
    // Professional Toolbar
    ImGui::Begin("Toolbar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
    ImGui::SetWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + ImGui::GetFrameHeight()));
    ImGui::SetWindowSize(ImVec2(viewport->WorkSize.x, 40));
    
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 0.0f));
    
    // Play/Stop button with better styling
    ImGui::SetCursorPosY(5);
    if (!state.play_mode) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
        if (ImGui::Button("  Play  ", ImVec2(70, 30))) {
            SceneSerializer::save("_temp_editor_state.txt", state.registry);
            state.play_mode = true;
            log_console("Entering Play mode");
        }
        ImGui::PopStyleColor(3);
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
        if (ImGui::Button("  Stop  ", ImVec2(70, 30))) {
            state.play_mode = false;
            std::vector<EntityId> to_delete;
            state.registry.transforms.each([&](EntityId e, Transform& t) {
                to_delete.push_back(e);
            });
            for (auto e : to_delete) {
                state.registry.destroy(e);
            }
            state.selected_entity = NULL_ENTITY;
            SceneSerializer::load("_temp_editor_state.txt", state.registry, state.world);
            log_console("Exiting Play mode");
        }
        ImGui::PopStyleColor(3);
    }
    
    ImGui::SameLine();
    ImGui::Text(" | ");
    ImGui::SameLine();
    ImGui::SetCursorPosY(12);
    
    // Stats
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::SameLine();
    ImGui::Text(" | ");
    ImGui::SameLine();
    ImGui::Text("Entities: %d", (int)state.registry.transforms.entities.size());
    ImGui::PopStyleColor();
    
    ImGui::PopStyleVar(2);
    ImGui::End();

    // 1. Hierarchy panel
    if (show_hierarchy && !state.play_mode) {
        ImGui::Begin("Hierarchy", &show_hierarchy);
        
        // Toolbar buttons
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 4.0f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.2f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.6f, 0.3f, 1.0f));
        if (ImGui::Button("+ Create", ImVec2(-1, 0))) {
            EntityId new_entity = state.registry.create();
            state.registry.transforms.add(new_entity, Transform());
            log_console("Created entity " + std::to_string(new_entity.id));
        }
        ImGui::PopStyleColor(2);
        
        if (state.selected_entity != NULL_ENTITY) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.2f, 0.2f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.3f, 0.3f, 1.0f));
            if (ImGui::Button("Delete", ImVec2(-1, 0))) {
                state.registry.destroy(state.selected_entity);
                log_console("Deleted entity " + std::to_string(state.selected_entity.id));
                state.selected_entity = NULL_ENTITY;
            }
            ImGui::PopStyleColor(2);
        }
        ImGui::PopStyleVar();
        
        ImGui::Separator();
        
        // Entity count header
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
        ImGui::Text("Entities (%d)", (int)state.registry.transforms.entities.size());
        ImGui::PopStyleColor();
        ImGui::Separator();
        
        // Entity list with better icons and styling
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));
        state.registry.transforms.each([&](EntityId e, Transform& t) {
            // Determine icon based on components (using simple ASCII)
            const char* icon = "[E]"; // Default entity
            if (state.registry.cameras.has(e)) icon = "[C]"; // Camera
            else if (state.registry.rigidbodies.has(e) && state.registry.sprites.has(e)) icon = "[P]"; // Physics + Sprite
            else if (state.registry.sprites.has(e)) icon = "[S]"; // Sprite
            else if (state.registry.rigidbodies.has(e)) icon = "[R]"; // Rigidbody only
            
            char label[128];
            snprintf(label, sizeof(label), "%s Entity_%u", icon, e.id);
            
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
            if (e == state.selected_entity) {
                flags |= ImGuiTreeNodeFlags_Selected;
                ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.26f, 0.59f, 0.98f, 0.4f));
                ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.26f, 0.59f, 0.98f, 0.6f));
            }
            
            ImGui::TreeNodeEx(label, flags);
            if (e == state.selected_entity) {
                ImGui::PopStyleColor(2);
            }
            
            if (ImGui::IsItemClicked()) {
                state.selected_entity = e;
            }
            
            // Note: No TreePop() needed when using ImGuiTreeNodeFlags_NoTreePushOnOpen and ImGuiTreeNodeFlags_Leaf
        });
        ImGui::PopStyleVar();
        
        ImGui::End();
    }
    
    // 2. Inspector panel
    if (show_inspector && !state.play_mode) {
        ImGui::Begin("Inspector", &show_inspector);
        
        if (state.selected_entity != NULL_ENTITY && state.registry.valid(state.selected_entity)) {
            // Entity header with ID
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.9f, 1.0f, 1.0f));
            ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[0]); // Use default font but could use bold
            ImGui::Text("Entity %u", state.selected_entity.id);
            ImGui::PopFont();
            ImGui::PopStyleColor();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
            ImGui::Text("Generation: %u", state.selected_entity.generation);
            ImGui::PopStyleColor();
            ImGui::Separator();
            
            // Transform component
            Transform* transform = state.registry.transforms.get(state.selected_entity);
            if (transform) {
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(8.0f);
                    ImGui::Text("Position");
                    ImGui::DragFloat2("##Position", &transform->position.X, 1.0f, -10000.0f, 10000.0f, "%.2f");
                    ImGui::Text("Rotation");
                    ImGui::DragFloat("##Rotation", &transform->rotation, 0.01f, -360.0f, 360.0f, "%.2f deg");
                    ImGui::Text("Scale");
                    ImGui::DragFloat2("##Scale", &transform->scale.X, 0.01f, 0.01f, 100.0f, "%.2f");
                    ImGui::Unindent(8.0f);
                }
                ImGui::PopStyleVar();
            }
            
            // Sprite component
            Sprite* sprite = state.registry.sprites.get(state.selected_entity);
            if (sprite) {
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                if (ImGui::CollapsingHeader("Sprite", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(8.0f);
                    ImGui::Text("Color");
                    ImGui::ColorEdit4("##Color", &sprite->color.X);
                    ImGui::Text("Size");
                    ImGui::DragFloat2("##Size", &sprite->size.X, 1.0f, 1.0f, 500.0f, "%.1f");
                    
                    // Texture loading
                    static char tex_path[256] = "";
                    ImGui::Text("Texture");
                    ImGui::PushItemWidth(-1);
                    ImGui::InputText("##TexPath", tex_path, sizeof(tex_path));
                    ImGui::PopItemWidth();
                    if (ImGui::Button("Browse...", ImVec2(-1, 0))) {
                        nfdchar_t* outPath = nullptr;
                        nfdfilteritem_t filters[1] = { { "Image", "png,jpg,jpeg" } };
                        nfdresult_t result = NFD_OpenDialog(&outPath, filters, 1, nullptr);
                        if (result == NFD_OKAY) {
                            strncpy(tex_path, outPath, sizeof(tex_path) - 1);
                            NFD_FreePath(outPath);
                        }
                    }
                    if (ImGui::Button("Load Texture", ImVec2(-1, 0))) {
                        sg_image img = AssetManager::load_texture(tex_path);
                        if (img.id != SG_INVALID_ID) {
                            sprite->texture = img;
                            log_console("Texture loaded: " + std::string(tex_path));
                        } else {
                            log_console("Failed to load texture: " + std::string(tex_path));
                        }
                    }
                    
                    bool has_tex = sprite->texture.id != SG_INVALID_ID;
                    ImGui::PushStyleColor(ImGuiCol_Text, has_tex ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
                    ImGui::Text(has_tex ? "  Texture: Loaded" : "  Texture: None");
                    ImGui::PopStyleColor();
                    ImGui::Unindent(8.0f);
                }
                ImGui::PopStyleVar();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.25f, 0.25f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                if (ImGui::Button("+ Add Sprite Component", ImVec2(-1, 0))) {
                    state.registry.sprites.add(state.selected_entity, Sprite());
                }
                ImGui::PopStyleColor(2);
            }
            
            // Rigidbody component
            Rigidbody* rb = state.registry.rigidbodies.get(state.selected_entity);
            if (rb) {
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                if (ImGui::CollapsingHeader("Rigidbody", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(8.0f);
                    bool has_body = b2Body_IsValid(rb->body);
                    ImGui::PushStyleColor(ImGuiCol_Text, has_body ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
                    ImGui::Text(has_body ? "  Box2D Body: Valid" : "  Box2D Body: None");
                    ImGui::PopStyleColor();
                    
                    // Body type
                    const char* body_types[] = { "Static", "Kinematic", "Dynamic" };
                    int current_type = (int)rb->body_type;
                    ImGui::Text("Body Type");
                    if (ImGui::Combo("##BodyType", &current_type, body_types, 3)) {
                        rb->body_type = (b2BodyType)current_type;
                        if (has_body) {
                            b2Body_SetType(rb->body, rb->body_type);
                        }
                    }
                    
                    // Fixed rotation
                    ImGui::Text("Fixed Rotation");
                    if (ImGui::Checkbox("##FixedRot", &rb->fixed_rotation)) {
                        if (has_body) {
                            b2Body_SetFixedRotation(rb->body, rb->fixed_rotation);
                        }
                    }
                    
                    // Physics properties
                    ImGui::Text("Density");
                    ImGui::DragFloat("##Density", &rb->density, 0.1f, 0.0f, 100.0f, "%.2f");
                    ImGui::Text("Friction");
                    ImGui::DragFloat("##Friction", &rb->friction, 0.01f, 0.0f, 1.0f, "%.2f");
                    ImGui::Text("Restitution");
                    ImGui::DragFloat("##Restitution", &rb->restitution, 0.01f, 0.0f, 1.0f, "%.2f");
                    
                    if (!has_body) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.8f, 0.8f));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.6f, 0.9f, 1.0f));
                        if (ImGui::Button("Create Box2D Body", ImVec2(-1, 0))) {
                            b2BodyDef bodyDef = b2DefaultBodyDef();
                            Transform* t = state.registry.transforms.get(state.selected_entity);
                            if (t) {
                                bodyDef.position = b2Vec2{t->position.X, t->position.Y};
                                bodyDef.rotation = b2MakeRot(t->rotation);
                            }
                            bodyDef.type = rb->body_type;
                            bodyDef.fixedRotation = rb->fixed_rotation;
                            rb->body = b2CreateBody(state.world, &bodyDef);
                            
                            // Add a box shape
                            Sprite* sprite = state.registry.sprites.get(state.selected_entity);
                            float hw = sprite ? sprite->size.X * 0.5f : 50.0f;
                            float hh = sprite ? sprite->size.Y * 0.5f : 50.0f;
                            b2Polygon box = b2MakeBox(hw, hh);
                            b2ShapeDef shapeDef = b2DefaultShapeDef();
                            shapeDef.density = rb->density;
                            shapeDef.material.friction = rb->friction;
                            shapeDef.material.restitution = rb->restitution;
                            b2CreatePolygonShape(rb->body, &shapeDef, &box);
                            
                            log_console("Created Box2D body for entity " + std::to_string(state.selected_entity.id));
                        }
                        ImGui::PopStyleColor(2);
                    }
                    ImGui::Unindent(8.0f);
                }
                ImGui::PopStyleVar();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.25f, 0.25f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                if (ImGui::Button("+ Add Rigidbody Component", ImVec2(-1, 0))) {
                    state.registry.rigidbodies.add(state.selected_entity, Rigidbody());
                }
                ImGui::PopStyleColor(2);
            }
            
            // Script component
            Script* script = state.registry.scripts.get(state.selected_entity);
            if (script) {
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                if (ImGui::CollapsingHeader("Script", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(8.0f);
                    char buf[256];
                    strncpy(buf, script->path.c_str(), sizeof(buf));
                    buf[sizeof(buf)-1] = '\0';
                    ImGui::Text("Script Path");
                    ImGui::PushItemWidth(-1);
                    if (ImGui::InputText("##ScriptPath", buf, sizeof(buf))) {
                        script->path = buf;
                        script->loaded = false; // Force reload
                    }
                    ImGui::PopItemWidth();
                    if (ImGui::Button("Browse...", ImVec2(-1, 0))) {
                        nfdchar_t* outPath = nullptr;
                        nfdfilteritem_t filters[1] = { { "Lua Script", "lua" } };
                        nfdresult_t result = NFD_OpenDialog(&outPath, filters, 1, nullptr);
                        if (result == NFD_OKAY) {
                            script->path = outPath;
                            script->loaded = false; // Force reload
                            NFD_FreePath(outPath);
                        }
                    }
                    ImGui::PushStyleColor(ImGuiCol_Text, script->loaded ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
                    ImGui::Text(script->loaded ? "  Status: Loaded" : "  Status: Not Loaded");
                    ImGui::PopStyleColor();
                    ImGui::Unindent(8.0f);
                }
                ImGui::PopStyleVar();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.25f, 0.25f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                if (ImGui::Button("+ Add Script Component", ImVec2(-1, 0))) {
                    state.registry.scripts.add(state.selected_entity, Script());
                }
                ImGui::PopStyleColor(2);
            }
            
            // Camera component
            Camera* camera = state.registry.cameras.get(state.selected_entity);
            if (camera) {
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(8.0f);
                    ImGui::Text("Zoom");
                    ImGui::DragFloat("##Zoom", &camera->zoom, 0.01f, 0.1f, 10.0f, "%.2f");
                    ImGui::Text("Offset");
                    ImGui::DragFloat2("##Offset", &camera->offset.X, 1.0f, -1000.0f, 1000.0f, "%.1f");
                    ImGui::Unindent(8.0f);
                }
                ImGui::PopStyleVar();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.25f, 0.25f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.35f, 0.35f, 1.0f));
                if (ImGui::Button("+ Add Camera Component", ImVec2(-1, 0))) {
                    state.registry.cameras.add(state.selected_entity, Camera());
                }
                ImGui::PopStyleColor(2);
            }
            
        } else {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 20.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
            ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize("No entity selected").x) * 0.5f);
            ImGui::Text("No entity selected");
            ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize("Select an entity in the Hierarchy or Viewport").x) * 0.5f);
            ImGui::Text("Select an entity in the Hierarchy or Viewport");
            ImGui::PopStyleColor();
        }
        
        ImGui::End();
    }
    
    // 3. Viewport window
    if (show_viewport) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", &show_viewport);
        ImGui::PopStyleVar();
        
        // Get viewport draw list and window bounds
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
        
        // Draw background with professional checkerboard pattern
        ImU32 bg_color1 = IM_COL32(30, 30, 30, 255);
        ImU32 bg_color2 = IM_COL32(35, 35, 35, 255);
        const float CHECKER_SIZE = 20.0f;
        for (float y = canvas_p0.y; y < canvas_p1.y; y += CHECKER_SIZE) {
            for (float x = canvas_p0.x; x < canvas_p1.x; x += CHECKER_SIZE) {
                bool checker = ((int)(x / CHECKER_SIZE) + (int)(y / CHECKER_SIZE)) % 2 == 0;
                dl->AddRectFilled(ImVec2(x, y), ImVec2(x + CHECKER_SIZE, y + CHECKER_SIZE), 
                                 checker ? bg_color1 : bg_color2);
            }
        }
        dl->AddRect(canvas_p0, canvas_p1, IM_COL32(60, 60, 60, 255), 0.0f, 0, 2.0f);
        
        // Professional grid
        const float GRID_STEP = 64.0f;
        ImU32 grid_color = IM_COL32(80, 80, 80, 60);
        for (float x = fmodf(canvas_p0.x, GRID_STEP); x < canvas_p1.x; x += GRID_STEP)
            dl->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), grid_color);
        for (float y = fmodf(canvas_p0.y, GRID_STEP); y < canvas_p1.y; y += GRID_STEP)
            dl->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), grid_color);
        
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
                
                // Professional selection highlight
                if (e == state.selected_entity) {
                    // Outer glow
                    dl->AddQuad(corners[0], corners[1], corners[2], corners[3], IM_COL32(100, 150, 255, 200), 3.0f);
                    // Inner border
                    dl->AddQuad(corners[0], corners[1], corners[2], corners[3], IM_COL32(200, 220, 255, 255), 1.5f);
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
    
    // 4. Console window
    if (show_console) {
        ImGui::Begin("Console", &show_console);
        
        // Console toolbar
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 4.0f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
        if (ImGui::Button("Clear", ImVec2(60, 0))) {
            console_logs.clear();
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
        ImGui::Text("Messages: %d", (int)console_logs.size());
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
        
        ImGui::Separator();
        
        // Console content with better styling
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));
        ImGui::BeginChild("ScrollingRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
        
        for (const auto& log : console_logs) {
            ImVec4 color = ImVec4(0.9f, 0.9f, 0.9f, 1.0f);
            if (log.find("[Lua]") != std::string::npos) {
                color = ImVec4(0.4f, 0.7f, 1.0f, 1.0f);
            } else if (log.find("Error") != std::string::npos || log.find("Failed") != std::string::npos) {
                color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
            } else if (log.find("Created") != std::string::npos || log.find("Entering") != std::string::npos || log.find("Loaded") != std::string::npos) {
                color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
            } else if (log.find("Warning") != std::string::npos) {
                color = ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
            }
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", log.c_str());
            ImGui::PopStyleColor();
        }
        
        // Auto-scroll to bottom
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5.0f)
            ImGui::SetScrollHereY(1.0f);
        
        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::End();
    }
    
    // 5. Assets window
    if (show_assets) {
        ImGui::Begin("Assets", &show_assets);
        
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
        ImGui::Text("Asset Browser");
        ImGui::PopStyleColor();
        ImGui::Separator();
        
        ImGui::Text("Textures: %d", (int)AssetManager::textures.size());
        if (!AssetManager::textures.empty()) {
            ImGui::Separator();
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));
            for (const auto& pair : AssetManager::textures) {
                ImGui::Text("  %s", pair.first.c_str());
            }
            ImGui::PopStyleVar();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
            ImGui::Text("No assets loaded");
            ImGui::PopStyleColor();
        }
        
        ImGui::End();
    }
    
    // Demo window
    if (show_test_window) {
        ImGui::ShowDemoWindow(&show_test_window);
    }

    sg_pass _sg_pass{};
    _sg_pass.action = state.pass_action;
    _sg_pass.swapchain = sglue_swapchain();

    sg_begin_pass(&_sg_pass);
    // removed: sgp_flush / sgp_end
    sgimgui_draw(&state.sgimgui);
    simgui_render();
    sg_end_pass();
    sg_commit();
}

void cleanup(void) {
    state.play_mode = false;
    std::vector<EntityId> to_delete;
    state.registry.transforms.each([&](EntityId e, Transform& t) {
        to_delete.push_back(e);
    });
    for (auto e : to_delete) {
        state.registry.destroy(e);
    }

    AssetManager::cleanup();
    sgimgui_discard(&state.sgimgui);
    simgui_shutdown();
    b2DestroyWorld(state.world);
    if (state.lua) { delete state.lua; state.lua = nullptr; }
    PHYSFS_deinit();
    NFD_Quit();
    sg_shutdown();
}

void event(const sapp_event* e) {
    simgui_handle_event(e);
    
    // F5 to toggle play mode
    if (e->type == SAPP_EVENTTYPE_KEY_DOWN && e->key_code == SAPP_KEYCODE_F5) {
        if (!state.play_mode) {
            SceneSerializer::save("_temp_editor_state.txt", state.registry);
            state.play_mode = true;
        } else {
            state.play_mode = false;
            std::vector<EntityId> to_delete;
            state.registry.transforms.each([&](EntityId e, Transform& t) {
                to_delete.push_back(e);
            });
            for (auto e : to_delete) {
                state.registry.destroy(e);
            }
            state.selected_entity = NULL_ENTITY;
            SceneSerializer::load("_temp_editor_state.txt", state.registry, state.world);
        }
    }
    
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
