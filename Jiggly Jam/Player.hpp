#pragma once

#include "libs.hpp"
#include "jelly.hpp"
#include <chrono>
#include <random>

enum class MOVE_TYPE
{
    UP,
    DOWN,
    LEFT,
    RIGHT,
    UP_RIGHT,
    UP_LEFT,
    DOWN_RIGHT,
    DOWN_LEFT
};
uint64_t generateID()
{
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t time = now.time_since_epoch().count();
    std::mt19937_64 rng(time);

    return rng();
}


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
    // continuous movement force (per-second style). kept small because impulses do the bulk of movement.
    float move_force = 20000.f;
    // instantaneous lateral impulse (used for snappy, dt-independent movement). Lowered for less sensitivity.
    float move_impulse = 400.f;
    float jump_force = 200.f;
    // desired steady walk speed when holding movement (pixels per physics step)
    float walk_speed = 2.2f;
    // tuning: increase move force for stronger lateral movement
    // you can tweak this at runtime or per-player for control feel
    // float move_force = 20000.f;
    float coyoteTimer = 0.f;
    // movement state for edge detection and momentum
    int last_move_sign = 0; // -1,0,1
    float air_drag = 4.f;   // per-second horizontal damping when not pressing
    void create_figure();
    // ground / contact tuning
    float ground_level = 600.f;
    float ground_restitution = 0.35f; // bounce
    float ground_friction = 0.6f;     // 0..1, larger = more friction (slows horizontal vel)
public:
    // public input enum (set this from your input manager keyed by player ID)
    MOVE_TYPE input = MOVE_TYPE::DOWN;
    // remember previous input so we can apply one-shot impulses on transitions
    MOVE_TYPE last_input = MOVE_TYPE::DOWN;

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

    // runtime
    void update(float dt);
    void set_onGround(bool g);
    // explicit edge-triggered jump request (call from input manager on button-down)
    void request_jump();

    Player(std::string c);
    ~Player();
};

Player::Player(std::string c) : velocity(0.f, 0.f), acceleration(0.f, 0.f), height(100.f), weight(70.f), alive(true), onGround(true), health(100.f), color(c)
{
    // for position need to create random position and place there!
    // position {random,random}
    ID = generateID();
    create_figure();
}

Player::~Player()
{
}
