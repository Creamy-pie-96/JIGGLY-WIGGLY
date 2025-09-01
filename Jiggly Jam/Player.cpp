#include "Player.hpp"
#include <algorithm>
#include "jelly.hpp"

class Human
{

public:
    float s;

private:
    // 1) explicit landmark positions (example; scale/shift as needed)
    sf::Vector2f origin{100.f, 100.f};
    float height;
    sf::Vector2f HEAD;
    sf::Vector2f NECK;
    sf::Vector2f SHO_R;
    sf::Vector2f ELB_R;
    sf::Vector2f HAND_R;
    sf::Vector2f SHO_L;
    sf::Vector2f ELB_L;
    sf::Vector2f HAND_L;
    sf::Vector2f WAIST_L;
    sf::Vector2f HIP_L;
    sf::Vector2f KNEE_L;
    sf::Vector2f FOOT_L;
    sf::Vector2f WAIST_R;
    sf::Vector2f HIP_R;
    sf::Vector2f KNEE_R;
    sf::Vector2f FOOT_R;

public:
    // Order these clockwise (perimeter)
    std::vector<std::pair<sf::Vector2f, BODY_PART>> outline;

    Human(const sf::Vector2f &orig, const float &h) : origin(orig), height(h)
    {
        s = height / 165.f; // keep same demo scale mapping
        HEAD = origin + sf::Vector2f(0.f, -70.f) * s;
        NECK = origin + sf::Vector2f(0.f, -50.f) * s;
        SHO_R = origin + sf::Vector2f(36.f, -30.f) * s;
        ELB_R = origin + sf::Vector2f(48.f, -5.f) * s;
        HAND_R = origin + sf::Vector2f(56.f, 10.f) * s;
        SHO_L = origin + sf::Vector2f(-36.f, -30.f) * s;
        ELB_L = origin + sf::Vector2f(-48.f, -5.f) * s;
        HAND_L = origin + sf::Vector2f(-56.f, 10.f) * s;
        WAIST_L = origin + sf::Vector2f(-28.f, 40.f) * s;
        HIP_L = origin + sf::Vector2f(-18.f, 40.f) * s;
        KNEE_L = origin + sf::Vector2f(-6.f, 80.f) * s;
        FOOT_L = origin + sf::Vector2f(-6.f, 95.f) * s;
        WAIST_R = origin + sf::Vector2f(18.f, 40.f) * s;
        HIP_R = origin + sf::Vector2f(6.f, 80.f) * s;
        KNEE_R = origin + sf::Vector2f(6.f, 95.f) * s;
        FOOT_R = origin + sf::Vector2f(2.f, 95.f) * s;

        outline =
            {
                {HEAD, BODY_PART::HEAD}, // Top of head
                {NECK, BODY_PART::NECK}, // Neck connection

                // Right arm
                {SHO_R, BODY_PART::SHO_R},
                {ELB_R, BODY_PART::ELB_R},
                {HAND_R, BODY_PART::HAND_R},

                // Right side torso & right leg
                {WAIST_R, BODY_PART::WAIST_R},
                {HIP_R, BODY_PART::HIP_R},
                {KNEE_R, BODY_PART::KNEE_R},
                {FOOT_R, BODY_PART::FOOT_R},

                // Left leg (going back up)
                {FOOT_L, BODY_PART::FOOT_L},
                {KNEE_L, BODY_PART::KNEE_L},
                {HIP_L, BODY_PART::HIP_L},
                {WAIST_L, BODY_PART::WAIST_L},

                // Left arm
                {HAND_L, BODY_PART::HAND_L},
                {ELB_L, BODY_PART::ELB_L},
                {SHO_L, BODY_PART::SHO_L},

                {NECK, BODY_PART::NECK}, // Return to neck
                {HEAD, BODY_PART::HEAD}  // Close loop at head
            };
    }
    ~Human() = default;
};
void Player::create_figure()
{
    Human human_shape({100.f, 150.f}, this->height);
    float spacing = std::max(6.f, 8.f * human_shape.s);
    figure.clear();
    figure.create_filled_from_polygon(human_shape.outline, spacing, this->ID);
}
void Player::spawn(sf::Vector2f pos)
{
    if (figure.points.empty())
        create_figure();

    // place center and peripheral points relative to new center
    sf::Vector2f oldCenter = figure.points.size() ? figure.points[0].pos : sf::Vector2f{0.f, 0.f};
    sf::Vector2f center = pos;
    figure.points[0].pos = center;
    figure.points[0].prev_pos = center;

    for (size_t i = 1; i < figure.points.size(); ++i)
    {
        sf::Vector2f dir = figure.points[i].pos - oldCenter;
        figure.points[i].pos = center + dir;
        figure.points[i].prev_pos = figure.points[i].pos;
    }

    // reset any state
    onGround = false;
    coyoteTimer = 0.f;
}

void Player::set_onGround(bool g)
{
    onGround = g;
    if (g)
        coyoteTimer = 0.12f;
}

void Player::request_jump()
{
    // mark requested; handled immediately to apply impulse once
    if (onGround || coyoteTimer > 0.f)
    {
        sf::Vector2f J = sf::Vector2f(0.f, -jump_force);
        // instantaneous impulse: pass dt=1 so the jelly code treats J as full impulse
        if (figure.points.size())
            figure.apply_force(J, figure.points[0].pos, figure.get_radius() * 1.5f, 1.f);
        onGround = false;
        coyoteTimer = 0.f;
    }
}

void Player::update(float dt)
{
    // interpret public `input` (input manager should set this per-player)
    sf::Vector2f moveForce{0.f, 0.f};
    bool wantJump = false;

    switch (input)
    {
    case MOVE_TYPE::LEFT:
        moveForce.x = -move_force;
        break;
    case MOVE_TYPE::RIGHT:
        moveForce.x = move_force;
        break;
    case MOVE_TYPE::UP:
        wantJump = true;
        break;
    case MOVE_TYPE::UP_LEFT:
        moveForce.x = -move_force;
        wantJump = true;
        break;
    case MOVE_TYPE::UP_RIGHT:
        moveForce.x = move_force;
        wantJump = true;
        break;
    default:
        break;
    }

    // momentum handling:
    // - apply one-shot instantaneous impulse when move starts
    // - apply a small continuous nudge while holding
    // - when input released, apply horizontal damping (air_drag) to slow momentum
    int move_sign = 0;
    if (input == MOVE_TYPE::LEFT || input == MOVE_TYPE::UP_LEFT)
        move_sign = -1;
    else if (input == MOVE_TYPE::RIGHT || input == MOVE_TYPE::UP_RIGHT)
        move_sign = 1;

    if (move_sign != 0 && figure.points.size())
    {
        // on transition (start pressing) apply a snappy impulse
        if (move_sign != last_move_sign)
        {
            sf::Vector2f impulse = sf::Vector2f(move_sign * move_impulse, 0.f);
            figure.apply_force(impulse, figure.points[0].pos, figure.get_radius() * 1.5f, 1.f);
        }

        // compute center-of-mass velocity (verlet velocity = pos - prev_pos)
        float M = 0.f;
        sf::Vector2f com_vel{0.f, 0.f};
        for (auto &p : figure.points)
        {
            if (p.locked)
                continue;
            sf::Vector2f v = p.pos - p.prev_pos;
            com_vel += v * p.mass;
            M += p.mass;
        }
        if (M > 1e-6f)
            com_vel /= M;

        // desired horizontal velocity while holding movement key
        float desired_vx = move_sign * walk_speed;
        float dvx = desired_vx - com_vel.x;

        // apply corrective impulse to move COM toward desired velocity
        // scale by responsiveness (0..1) to avoid overshoot and jitter
        const float corrective_factor = 0.9f; // tuned for snappy but stable correction
        if (std::abs(dvx) > 1e-4f && M > 0.f)
        {
            float Jx = M * dvx * corrective_factor;
            figure.apply_force(sf::Vector2f(Jx, 0.f), figure.points[0].pos, figure.get_radius() * 1.5f, 1.f);
        }

        // small residual nudges to keep soft-body peripheral motion (very small, dt-scaled)
        figure.apply_force(sf::Vector2f(move_sign * move_force * dt * 0.005f, 0.f), figure.points[0].pos, figure.get_radius() * 2.f);
    }
    else
    {
        // no lateral input: apply horizontal damping to each point's velocity
        float damp = std::exp(-air_drag * dt); // smoother exponential decay
        for (auto &p : figure.points)
        {
            if (p.locked)
                continue;
            sf::Vector2f vel = p.pos - p.prev_pos;
            vel.x *= damp;
            p.prev_pos = p.pos - vel;
        }
    }

    // remember state for next frame
    last_move_sign = move_sign;
    last_input = input;

    // jumping: use impulse overload. The input manager should set MOVE_TYPE::UP only on edge
    if (wantJump && (onGround || coyoteTimer > 0.f))
    {
        // impulse magnitude tuned similarly to demo
        sf::Vector2f J = sf::Vector2f(0.f, -jump_force);
        // pass physics dt here as demo_jelly does when triggering from event; tuning may be needed
        figure.apply_force(J, figure.points[0].pos, figure.get_radius() * 1.5f, dt);
        onGround = false;
        coyoteTimer = 0.f;
    }

    if (coyoteTimer > 0.f)
        coyoteTimer -= dt;

    // integrate and satisfy constraints
    figure.update(dt);

    // simple ground collision: clamp peripheral points just above ground and damp vertical velocity
    if (figure.points.size() > 1)
    {
        for (size_t i = 1; i < figure.points.size(); ++i)
        {
            auto &p = figure.points[i];
            float minY = ground_level;
            if (p.pos.y > minY)
            {
                // project onto ground
                p.pos.y = minY;
                // damp vertical velocity (simulate restitution)
                p.prev_pos.y = p.pos.y + (p.prev_pos.y - p.pos.y) * (-ground_restitution);
                // horizontal friction
                p.prev_pos.x = p.pos.x + (p.prev_pos.x - p.pos.x) * (1.f - ground_friction);
                onGround = true;
            }
        }
    }
}
