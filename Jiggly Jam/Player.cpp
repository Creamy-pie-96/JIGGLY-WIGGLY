#include "Player.hpp"
#include <algorithm>
#include "jelly.hpp"

class Human
{
public:
    float s;

private:
    sf::Vector2f origin{100.f, 100.f};
    float height;

    // Key skeleton landmarks (these will be placed strategically in the outline)
    sf::Vector2f HEAD, NECK, SPINE_UP, SPINE_MID, SPINE_LOW, PELVIS;
    sf::Vector2f CLAV_R, CLAV_L;
    sf::Vector2f SHO_R, SHO_L;
    sf::Vector2f ELB_R, ELB_L;
    sf::Vector2f WRIST_R, WRIST_L;
    sf::Vector2f HAND_R, HAND_L;
    sf::Vector2f HIP_R, HIP_L;
    sf::Vector2f KNEE_R, KNEE_L;
    sf::Vector2f ANKLE_R, ANKLE_L;
    sf::Vector2f FOOT_R, FOOT_L;

public:
    std::vector<std::pair<sf::Vector2f, BODY_PART>> outline;
    std::vector<std::pair<sf::Vector2f, BODY_PART>> interior_joints; // For spine segments

    Human(const sf::Vector2f &orig, const float &h) : origin(orig), height(h)
    {
        s = height / 180.f; // More realistic human height proportions

        // Define canonical skeleton positions
        HEAD = origin + sf::Vector2f(0.f, -80.f) * s;
        NECK = origin + sf::Vector2f(0.f, -65.f) * s;
        SPINE_UP = origin + sf::Vector2f(0.f, -45.f) * s;  // Upper chest
        SPINE_MID = origin + sf::Vector2f(0.f, -20.f) * s; // Mid torso
        SPINE_LOW = origin + sf::Vector2f(0.f, 5.f) * s;   // Lower back
        PELVIS = origin + sf::Vector2f(0.f, 20.f) * s;     // Pelvis center

        CLAV_R = origin + sf::Vector2f(15.f, -60.f) * s;  // Right clavicle
        CLAV_L = origin + sf::Vector2f(-15.f, -60.f) * s; // Left clavicle
        SHO_R = origin + sf::Vector2f(25.f, -55.f) * s;   // Right shoulder
        SHO_L = origin + sf::Vector2f(-25.f, -55.f) * s;  // Left shoulder
        ELB_R = origin + sf::Vector2f(35.f, -25.f) * s;   // Right elbow
        ELB_L = origin + sf::Vector2f(-35.f, -25.f) * s;  // Left elbow
        WRIST_R = origin + sf::Vector2f(38.f, 0.f) * s;   // Right wrist
        WRIST_L = origin + sf::Vector2f(-38.f, 0.f) * s;  // Left wrist
        HAND_R = origin + sf::Vector2f(40.f, 5.f) * s;    // Right hand
        HAND_L = origin + sf::Vector2f(-40.f, 5.f) * s;   // Left hand

        HIP_R = origin + sf::Vector2f(12.f, 25.f) * s;    // Right hip
        HIP_L = origin + sf::Vector2f(-12.f, 25.f) * s;   // Left hip
        KNEE_R = origin + sf::Vector2f(15.f, 60.f) * s;   // Right knee
        KNEE_L = origin + sf::Vector2f(-15.f, 60.f) * s;  // Left knee
        ANKLE_R = origin + sf::Vector2f(12.f, 85.f) * s;  // Right ankle
        ANKLE_L = origin + sf::Vector2f(-12.f, 85.f) * s; // Left ankle
        FOOT_R = origin + sf::Vector2f(12.f, 90.f) * s;   // Right foot
        FOOT_L = origin + sf::Vector2f(-12.f, 90.f) * s;  // Left foot

        // Create human silhouette with strategic skeleton joint placement
        outline = {
            // Head
            {HEAD, BODY_PART::HEAD},
            {origin + sf::Vector2f(8.f, -75.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(12.f, -68.f) * s, BODY_PART::NONE},
            {NECK, BODY_PART::NECK},

            // Right side
            {CLAV_R, BODY_PART::CLAV_R},
            {SHO_R, BODY_PART::SHO_R},
            {origin + sf::Vector2f(30.f, -45.f) * s, BODY_PART::NONE},
            {ELB_R, BODY_PART::ELB_R},
            {origin + sf::Vector2f(37.f, -10.f) * s, BODY_PART::NONE},
            {WRIST_R, BODY_PART::WRIST_R},
            {HAND_R, BODY_PART::HAND_R},

            // Right torso and leg
            {origin + sf::Vector2f(35.f, -5.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(28.f, 10.f) * s, BODY_PART::NONE},
            {HIP_R, BODY_PART::HIP_R},
            {origin + sf::Vector2f(17.f, 40.f) * s, BODY_PART::NONE},
            {KNEE_R, BODY_PART::KNEE_R},
            {origin + sf::Vector2f(14.f, 75.f) * s, BODY_PART::NONE},
            {ANKLE_R, BODY_PART::ANKLE_R},
            {FOOT_R, BODY_PART::FOOT_R},

            // Bottom
            {origin + sf::Vector2f(8.f, 92.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(0.f, 93.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(-8.f, 92.f) * s, BODY_PART::NONE},

            // Left leg
            {FOOT_L, BODY_PART::FOOT_L},
            {ANKLE_L, BODY_PART::ANKLE_L},
            {origin + sf::Vector2f(-14.f, 75.f) * s, BODY_PART::NONE},
            {KNEE_L, BODY_PART::KNEE_L},
            {origin + sf::Vector2f(-17.f, 40.f) * s, BODY_PART::NONE},
            {HIP_L, BODY_PART::HIP_L},

            // Left torso and arm
            {origin + sf::Vector2f(-28.f, 10.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(-35.f, -5.f) * s, BODY_PART::NONE},
            {HAND_L, BODY_PART::HAND_L},
            {WRIST_L, BODY_PART::WRIST_L},
            {origin + sf::Vector2f(-37.f, -10.f) * s, BODY_PART::NONE},
            {ELB_L, BODY_PART::ELB_L},
            {origin + sf::Vector2f(-30.f, -45.f) * s, BODY_PART::NONE},
            {SHO_L, BODY_PART::SHO_L},
            {CLAV_L, BODY_PART::CLAV_L},

            // Left side of head
            {origin + sf::Vector2f(-12.f, -68.f) * s, BODY_PART::NONE},
            {origin + sf::Vector2f(-8.f, -75.f) * s, BODY_PART::NONE},
        };

        // Interior skeleton joints (spine segments and pelvis center)
        interior_joints = {
            {SPINE_UP, BODY_PART::SPINE_UP},
            {SPINE_MID, BODY_PART::SPINE_MID},
            {SPINE_LOW, BODY_PART::SPINE_LOW},
            {PELVIS, BODY_PART::PELVIS}};
    }
    ~Human() = default;
};
void Player::create_figure()
{
    Human human_shape({100.f, 150.f}, this->height);
    float spacing = std::max(6.f, 8.f * human_shape.s);
    figure.clear();
    figure.create_filled_from_polygon(human_shape.outline, spacing, this->ID);

    // Add interior skeleton joints manually
    for (const auto &joint : human_shape.interior_joints)
    {
        Point p;
        p.pos = joint.first;
        p.prev_pos = joint.first;
        p.mass = 1.f;
        p.locked = false;
        p.body_part = joint.second;
        p.id = this->ID;
        figure.points.push_back(p);
    }

    // add skeleton springs (strong) and flesh-to-skeleton connections (soft)
    figure.add_skeleton_for_player(this->ID, 2.0f); // stronger skeleton stiffness
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
