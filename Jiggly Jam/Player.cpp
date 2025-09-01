#include "Player.hpp"
#include <algorithm>

// Player implementation guided by demo_jelly.cpp patterns.
// Movement applies a local force each physics step (scaled by dt).
// Jump uses the impulse overload so the jelly receives an instantaneous velocity change.

void Player::create_figure()
{
    // Build a human-ish outline identical to demo_jelly.cpp, scaled by `height`.
    // demo_jelly's vertical span is ~165px; map `height` to that span.
    float demo_span = 165.f;
    float scale = (height > 0.f) ? (height / demo_span) : 1.f;
    sf::Vector2f pos{100.f, 100.f};

    std::vector<sf::Vector2f> outline;
    outline.reserve(16);

    outline.push_back(pos + sf::Vector2f(-18.f, -70.f) * scale);
    outline.push_back(pos + sf::Vector2f(18.f, -70.f) * scale);
    outline.push_back(pos + sf::Vector2f(36.f, -30.f) * scale);
    outline.push_back(pos + sf::Vector2f(36.f, 0.f) * scale);
    outline.push_back(pos + sf::Vector2f(18.f, 40.f) * scale);
    outline.push_back(pos + sf::Vector2f(6.f, 80.f) * scale);
    outline.push_back(pos + sf::Vector2f(2.f, 95.f) * scale);
    outline.push_back(pos + sf::Vector2f(-6.f, 95.f) * scale);
    outline.push_back(pos + sf::Vector2f(-10.f, 80.f) * scale);
    outline.push_back(pos + sf::Vector2f(-28.f, 40.f) * scale);
    outline.push_back(pos + sf::Vector2f(-36.f, 0.f) * scale);
    outline.push_back(pos + sf::Vector2f(-36.f, -30.f) * scale);

    // choose spacing relative to player size (smaller spacing = denser lattice)
    float spacing = std::max(6.f, 8.f * scale);

    // create filled soft-body using new factory
    figure.clear();
    figure.create_filled_from_polygon(outline, spacing);

    // tune defaults for player feel (adjusted by weight/height)
    // figure.stiffness = 0.85f;
    // figure.stiffness_ring = 1.0f;
    // figure.stiffness_spoke = 1.0f;
    // damping: heavier players should be less bouncy
    figure.damping = std::clamp(0.985f - (weight - 70.f) * 0.0008f, 0.94f, 0.998f);
    // pressure: mild baseline, slightly lower for heavier players
    figure.pressure = std::clamp(0.6f - (weight - 70.f) * 0.0025f, 0.2f, 1.2f);
    figure.iterations = 10;

    // scale masses with player weight while preserving ratios
    if (figure.points.size())
    {
        figure.points[0].locked = false;
        float massScale = (weight > 0.f) ? (weight / 70.f) : 1.f;
        figure.points[0].mass = 2.f * massScale;
        for (size_t i = 1; i < figure.points.size(); ++i)
            figure.points[i].mass = 1.f * massScale;
    }
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
