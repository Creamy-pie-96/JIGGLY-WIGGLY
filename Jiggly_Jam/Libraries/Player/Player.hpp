#pragma once

#include "../libs.hpp"
#include "../Gmae_physics/jelly.hpp"
#include "../Controll_system/BodyControlSystem.hpp"
#include "../settings_perser.hpp"
#include <chrono>
#include <random>
#include <memory>

enum class MOVE_TYPE
{
    UP,
    DOWN,
    LEFT,
    RIGHT,
    UP_RIGHT,
    UP_LEFT,
    DOWN_RIGHT,
    DOWN_LEFT,
    NONE // Added for no input state
};

uint64_t generateID();

class Player
{
protected:
    sf::Vector2f velocity, acceleration;
    float height, weight;
    uint64_t ID;
    Jelly figure;
    std::string color;
    bool onGround;
    float health;
    bool alive;

    // ENHANCED: Increased forces for better movement responsiveness
    float move_force = 25.f;   // Increased from 15.f for better movement
    float move_impulse = 40.f; // Increased from 25.f for better responsiveness
    float jump_force = 120.f;  // Increased for stronger jump height (was 75.f)
    float walk_speed = 2.2f;
    float coyoteTimer = 0.f;
    int last_move_sign = 0; // -1,0,1
    float air_drag = 4.f;   // per-second horizontal damping when not pressing

    // Advanced body control system
    std::unique_ptr<BodyControlSystem> bodyControlSystem;
    bool useAdvancedControls = true; // Toggle between simple and advanced controls

    // 🎮 GANG BEASTS EVOLUTION: Settings Integration
    std::unique_ptr<SettingsParser> settingsParser;
    bool gangBeastsPhysicsEnabled = true; // Toggle for Gang Beasts vs original physics

    // 🚶 MOVEMENT ANIMATION SYSTEM
    bool isWalking = false;
    bool isWaving = false;
    float walkTimer = 0.0f;
    float waveTimer = 0.0f;
    float walkCycleSpeed = 3.0f;   // walking frequency
    float waveDuration = 2.5f;     // wave duration
    float walkDirection = 1.0f;    // 1.0f = right, -1.0f = left
    float walkForwardSpeed = 8.0f; // PHYSICS FIX: Reduced from 25.0f to prevent excessive momentum

public:
    // 🎯 STABLE PHYSICS TIMESTEP - The Sweet Spot for Structural Integrity!
    static constexpr float STABLE_TIMESTEP = 1.0f / 120.0f; // 120 FPS - Perfect for stability

private:
    void create_figure();
    // ground / contact tuning
    float ground_level = 600.f;
    float ground_restitution = 0.35f; // bounce
    float ground_friction = 0.6f;     // 0..1, larger = more friction (slows horizontal vel)

public:
    MOVE_TYPE input = MOVE_TYPE::NONE;
    MOVE_TYPE last_input = MOVE_TYPE::NONE;

    // Constructor that takes window for input system
    Player(std::string c, sf::RenderWindow *window = nullptr);
    ~Player();

    // Control system management
    void setAdvancedControls(bool enabled) { useAdvancedControls = enabled; }
    bool isUsingAdvancedControls() const { return useAdvancedControls; }
    BodyControlSystem *getControlSystem() const { return bodyControlSystem.get(); }

    // 🎮 GANG BEASTS EVOLUTION: Settings Management
    bool loadGangBeastsSettings(const std::string &settingsPath = "Game Settings/settings.json");
    void enableGangBeastsPhysics(bool enabled) { gangBeastsPhysicsEnabled = enabled; }
    bool isGangBeastsPhysicsEnabled() const { return gangBeastsPhysicsEnabled; }
    const GangBeastsSettings::GangBeastsPhysicsSettings *getGangBeastsSettings() const;
    void reloadSettings()
    {
        if (settingsParser)
            settingsParser->reloadSettings();
    }
    void printCurrentSettings() const
    {
        if (settingsParser)
            settingsParser->printSettings();
    }

    // setters for external input manager
    void set_input(MOVE_TYPE m) { input = m; }
    void set_ground_level(float y) { ground_level = y; }
    void set_ground_params(float restitution, float friction)
    {
        ground_restitution = restitution;
        ground_friction = friction;
    }

    uint64_t get_id() { return ID; }
    std::string get_color() { return color; }
    float get_health() { return health; }
    bool is_alive() { return alive; }
    bool is_onGround() { return onGround; }
    float get_move_force() { return move_force; }
    float get_jump_force() { return jump_force; }
    Jelly Figure() { return figure; }
    void spawn(sf::Vector2f pos);
    sf::Vector2f get_position() const { return figure.points.size() ? figure.points[0].pos : sf::Vector2f{0.f, 0.f}; }
    size_t getPointCount() const { return figure.points.size(); }

    // runtime
    void update(float dt);
    void updateSimpleControls(float dt); // Legacy control system
    void updatePhysics(float dt);        // Common physics update
    void set_onGround(bool g);
    void request_jump();

    // 🚶 MOVEMENT ANIMATIONS
    void startWalking()
    {
        isWalking = true;
        walkTimer = 0.0f;
    }
    void stopWalking()
    {
        isWalking = false;
        clearWalkTargets();
    }
    void setWalkDirection(float dir) { walkDirection = dir; } // 1.0f = right, -1.0f = left
    void startWaving()
    {
        isWaving = true;
        waveTimer = 0.0f;
    }
    void stopWaving()
    {
        isWaving = false;
        clearWaveTargets();
        // Restore normal skeleton spring strength after waving
        figure.animate_skeleton_spring(ID, BODY_PART::SHO_R, BODY_PART::ELB_R, 40.0f, 1.0f);    // Full strength
        figure.animate_skeleton_spring(ID, BODY_PART::ELB_R, BODY_PART::WRIST_R, 30.0f, 1.0f);  // Full strength
        figure.animate_skeleton_spring(ID, BODY_PART::WRIST_R, BODY_PART::HAND_R, 20.0f, 1.0f); // Full strength
    }

private:
    void updateWalkingAnimation(float dt);
    void updateWavingAnimation(float dt);
    void clearWalkTargets();
    void clearWaveTargets();

public:
    void drawControlDebug(sf::RenderWindow &window);
};

inline Player::Player(std::string c, sf::RenderWindow *window)
    : velocity(0.f, 0.f), acceleration(0.f, 0.f), height(100.f), weight(70.f),
      alive(true), onGround(true), health(100.f), color(c)
{
    ID = generateID();
    create_figure();

    if (window != nullptr)
    {
        bodyControlSystem = std::make_unique<BodyControlSystem>(window);
        std::cout << "🎮 Player " << ID << " initialized with advanced controls!" << std::endl;
    }
    else
    {
        useAdvancedControls = false;
        std::cout << "🎮 Player " << ID << " initialized with simple controls!" << std::endl;
    }

    // 🎮 GANG BEASTS EVOLUTION: Initialize settings system
    settingsParser = std::make_unique<SettingsParser>();
    loadGangBeastsSettings(); // Attempt to load settings (will use defaults if file not found)
}

inline Player::~Player()
{
}

inline void Player::drawControlDebug(sf::RenderWindow &window)
{
    if (useAdvancedControls && bodyControlSystem)
    {
        bodyControlSystem->draw(window);
    }
}

class PlayerControl : public Player
{
public:
    using Player::Player; // inherit constructor

    // wave demo state
    bool waving = false;
    float waveTimer = 0.f;
    float waveDuration = 2.5f; // seconds

    void start_wave()
    {
        waving = true;
        waveTimer = 0.f;
    }

    void update(float dt)
    {
        if (waving && figure.points.size())
        {
            int sho = figure.find_part_index(ID, BODY_PART::SHO_R);
            int hand = figure.find_part_index(ID, BODY_PART::HAND_R);
            int elb = figure.find_part_index(ID, BODY_PART::ELB_R);
            if (sho > 0 && hand > 0)
            {
                // base shoulder pos
                sf::Vector2f spos = figure.points[sho].pos;
                sf::Vector2f hpos = figure.points[hand].pos;
                // wave motion: circular offset around current hand position
                float freq = 6.f; // Hz
                float amp = 18.f; // pixels
                waveTimer += dt;
                float angle = std::sin(waveTimer * freq) * 0.8f;
                sf::Vector2f offset = sf::Vector2f(std::cos(angle), std::sin(angle)) * amp;
                sf::Vector2f target = spos + sf::Vector2f(40.f, -10.f) + offset; // reach out from shoulder
                // set PD target on hand
                figure.set_part_target(ID, BODY_PART::HAND_R, target, 300.f, 30.f);
                // also nudge elbow via animated spring to look like bending
                if (elb > 0)
                {
                    // find current spring length between SHO_R and ELB_R and oscillate
                    float baseLen = std::hypot(figure.points[elb].pos.x - figure.points[sho].pos.x, figure.points[elb].pos.y - figure.points[sho].pos.y);
                    float newRest = baseLen * (1.f - 0.08f * std::sin(waveTimer * freq * 0.5f));
                    figure.animate_skeleton_spring(ID, BODY_PART::SHO_R, BODY_PART::ELB_R, newRest, 0.5f);
                }
                if (waveTimer > waveDuration)
                {
                    waving = false;
                    figure.clear_part_target(ID, BODY_PART::HAND_R);
                }
            }
        }

        // call base update to handle movement and physics
        Player::update(dt);
    }
};
