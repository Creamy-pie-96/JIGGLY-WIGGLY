#include "Player.hpp"
#include <algorithm>
#include "../Gmae_physics/jelly.hpp"

class Human
{
public:
    float s;

private:
    sf::Vector2f origin{100.f, 100.f};
    float height;

    // Shape-based human figure (like in human.cpp)
    sf::RectangleShape torso;
    sf::RectangleShape neck;
    sf::CircleShape head;
    sf::RectangleShape rightArm;
    sf::RectangleShape leftArm;
    sf::RectangleShape rightHand;
    sf::RectangleShape leftHand;
    sf::RectangleShape rightLeg;
    sf::RectangleShape leftLeg;
    sf::RectangleShape rightFoot;
    sf::RectangleShape leftFoot;

    // Helper function to sample points around a circle
    std::vector<sf::Vector2f> sampleCircle(const sf::CircleShape &circle, int numPoints) const
    {
        std::vector<sf::Vector2f> points;
        sf::Vector2f center = circle.getPosition() + sf::Vector2f(circle.getRadius(), circle.getRadius());
        float radius = circle.getRadius();

        for (int i = 0; i < numPoints; ++i)
        {
            float angle = (float)i / (float)numPoints * 2.f * M_PI;
            sf::Vector2f point = center + sf::Vector2f(std::cos(angle), std::sin(angle)) * radius;
            points.push_back(point);
        }
        return points;
    }

    // Helper function to sample points around a rectangle
    std::vector<sf::Vector2f> sampleRectangle(const sf::RectangleShape &rect, int pointsPerSide) const
    {
        std::vector<sf::Vector2f> points;
        sf::Vector2f pos = rect.getPosition();
        sf::Vector2f size = rect.getSize();
        sf::Vector2f origin = rect.getOrigin();
        float rotation = rect.getRotation() * M_PI / 180.f;

        // Calculate the four corners relative to origin
        std::vector<sf::Vector2f> corners = {
            {-origin.x, -origin.y},                 // Top-left
            {size.x - origin.x, -origin.y},         // Top-right
            {size.x - origin.x, size.y - origin.y}, // Bottom-right
            {-origin.x, size.y - origin.y}          // Bottom-left
        };

        // Apply rotation and translation
        for (auto &corner : corners)
        {
            float x = corner.x * std::cos(rotation) - corner.y * std::sin(rotation);
            float y = corner.x * std::sin(rotation) + corner.y * std::cos(rotation);
            corner = sf::Vector2f(x + pos.x, y + pos.y);
        }

        // Sample points along each edge
        for (int edge = 0; edge < 4; ++edge)
        {
            sf::Vector2f start = corners[edge];
            sf::Vector2f end = corners[(edge + 1) % 4];

            for (int i = 0; i < pointsPerSide; ++i)
            {
                float t = (float)i / (float)pointsPerSide;
                sf::Vector2f point = start + (end - start) * t;
                points.push_back(point);
            }
        }
        return points;
    }

    // Helper function to determine which body part a point belongs to
    BODY_PART classifyPoint(const sf::Vector2f &point) const
    {
        // Check distance to canonical joint positions
        struct JointInfo
        {
            sf::Vector2f pos;
            BODY_PART part;
            float radius;
        };

        std::vector<JointInfo> joints = {
            {head.getPosition() + sf::Vector2f(head.getRadius(), head.getRadius()), BODY_PART::HEAD, head.getRadius() * 0.8f},
            {neck.getPosition() + neck.getSize() * 0.5f, BODY_PART::NECK, 10.f * s},
            {sf::Vector2f(neck.getPosition().x + neck.getSize().x * 0.8f, neck.getPosition().y + neck.getSize().y), BODY_PART::CLAV_R, 8.f * s},
            {sf::Vector2f(neck.getPosition().x + neck.getSize().x * 0.2f, neck.getPosition().y + neck.getSize().y), BODY_PART::CLAV_L, 8.f * s},
            {rightArm.getPosition(), BODY_PART::SHO_R, 12.f * s},
            {leftArm.getPosition(), BODY_PART::SHO_L, 12.f * s},
            {sf::Vector2f(rightArm.getPosition().x, rightArm.getPosition().y + rightArm.getSize().y * 0.6f), BODY_PART::ELB_R, 10.f * s},
            {sf::Vector2f(leftArm.getPosition().x, leftArm.getPosition().y + leftArm.getSize().y * 0.6f), BODY_PART::ELB_L, 10.f * s},
            {sf::Vector2f(rightArm.getPosition().x, rightArm.getPosition().y + rightArm.getSize().y * 0.9f), BODY_PART::WRIST_R, 8.f * s},
            {sf::Vector2f(leftArm.getPosition().x, leftArm.getPosition().y + leftArm.getSize().y * 0.9f), BODY_PART::WRIST_L, 8.f * s},
            {rightHand.getPosition() + rightHand.getSize() * 0.5f, BODY_PART::HAND_R, 12.f * s},
            {leftHand.getPosition() + leftHand.getSize() * 0.5f, BODY_PART::HAND_L, 12.f * s},
            {rightLeg.getPosition(), BODY_PART::HIP_R, 12.f * s},
            {leftLeg.getPosition(), BODY_PART::HIP_L, 12.f * s},
            {sf::Vector2f(rightLeg.getPosition().x, rightLeg.getPosition().y + rightLeg.getSize().y * 0.6f), BODY_PART::KNEE_R, 10.f * s},
            {sf::Vector2f(leftLeg.getPosition().x, leftLeg.getPosition().y + leftLeg.getSize().y * 0.6f), BODY_PART::KNEE_L, 10.f * s},
            {sf::Vector2f(rightLeg.getPosition().x, rightLeg.getPosition().y + rightLeg.getSize().y * 0.9f), BODY_PART::ANKLE_R, 8.f * s},
            {sf::Vector2f(leftLeg.getPosition().x, leftLeg.getPosition().y + leftLeg.getSize().y * 0.9f), BODY_PART::ANKLE_L, 8.f * s},
            {rightFoot.getPosition() + rightFoot.getSize() * 0.5f, BODY_PART::FOOT_R, 12.f * s},
            {leftFoot.getPosition() + leftFoot.getSize() * 0.5f, BODY_PART::FOOT_L, 12.f * s}};

        // Find the closest joint
        float minDist = 1e9f;
        BODY_PART closestPart = BODY_PART::NONE;

        for (const auto &joint : joints)
        {
            float dist = std::hypot(point.x - joint.pos.x, point.y - joint.pos.y);
            if (dist < joint.radius && dist < minDist)
            {
                minDist = dist;
                closestPart = joint.part;
            }
        }

        return closestPart;
    }

    // Main helper function to generate outline from shape-based figure
    void generateOutlineFromShapes()
    {
        std::vector<sf::Vector2f> allPoints;

        // Sample points from each body part
        auto headPoints = sampleCircle(head, 8);
        auto neckPoints = sampleRectangle(neck, 3);
        auto torsoPoints = sampleRectangle(torso, 6);
        auto rightArmPoints = sampleRectangle(rightArm, 4);
        auto leftArmPoints = sampleRectangle(leftArm, 4);
        auto rightHandPoints = sampleRectangle(rightHand, 3);
        auto leftHandPoints = sampleRectangle(leftHand, 3);
        auto rightLegPoints = sampleRectangle(rightLeg, 4);
        auto leftLegPoints = sampleRectangle(leftLeg, 4);
        auto rightFootPoints = sampleRectangle(rightFoot, 3);
        auto leftFootPoints = sampleRectangle(leftFoot, 3);

        // Combine all points
        allPoints.insert(allPoints.end(), headPoints.begin(), headPoints.end());
        allPoints.insert(allPoints.end(), neckPoints.begin(), neckPoints.end());
        allPoints.insert(allPoints.end(), torsoPoints.begin(), torsoPoints.end());
        allPoints.insert(allPoints.end(), rightArmPoints.begin(), rightArmPoints.end());
        allPoints.insert(allPoints.end(), leftArmPoints.begin(), leftArmPoints.end());
        allPoints.insert(allPoints.end(), rightHandPoints.begin(), rightHandPoints.end());
        allPoints.insert(allPoints.end(), leftHandPoints.begin(), leftHandPoints.end());
        allPoints.insert(allPoints.end(), rightLegPoints.begin(), rightLegPoints.end());
        allPoints.insert(allPoints.end(), leftLegPoints.begin(), leftLegPoints.end());
        allPoints.insert(allPoints.end(), rightFootPoints.begin(), rightFootPoints.end());
        allPoints.insert(allPoints.end(), leftFootPoints.begin(), leftFootPoints.end());

        // Compute the convex hull or use a simpler approach: sort by angle from center
        sf::Vector2f center{0.f, 0.f};
        for (const auto &p : allPoints)
            center += p;
        center /= (float)allPoints.size();

        // Sort points by angle around center to create a proper outline
        std::sort(allPoints.begin(), allPoints.end(), [center](const sf::Vector2f &a, const sf::Vector2f &b)
                  {
            float angleA = std::atan2(a.y - center.y, a.x - center.x);
            float angleB = std::atan2(b.y - center.y, b.x - center.x);
            return angleA < angleB; });

        // Create outline with body part classification
        outline.clear();
        for (const auto &point : allPoints)
        {
            BODY_PART part = classifyPoint(point);
            outline.push_back({point, part});
        }

        // Generate interior skeleton joints
        generateInteriorJoints();
    }

    void generateInteriorJoints()
    {
        // Calculate interior spine positions based on torso
        sf::Vector2f torsoCenter = torso.getPosition() + torso.getSize() * 0.5f;
        sf::Vector2f neckBottom = neck.getPosition() + sf::Vector2f(neck.getSize().x * 0.5f, neck.getSize().y);

        interior_joints = {
            {sf::Vector2f(torsoCenter.x, neckBottom.y + torso.getSize().y * 0.2f), BODY_PART::SPINE_UP},
            {torsoCenter, BODY_PART::SPINE_MID},
            {sf::Vector2f(torsoCenter.x, torsoCenter.y + torso.getSize().y * 0.3f), BODY_PART::SPINE_LOW},
            {sf::Vector2f(torsoCenter.x, torso.getPosition().y + torso.getSize().y), BODY_PART::PELVIS}};
    }

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
        s = h / 175.f; // Scale factor based on height (matching human.cpp)

        // Create shape-based human figure (similar to human.cpp)
        float headRadius = 15.f * s;
        float limbWidth = 8.f * s;
        float torsoWidth = 25.f * s;
        float torsoHeight = 50.f * s;
        float limbLength = 50.f * s;
        float handFootSize = 10.f * s;

        // Head and Neck
        head.setRadius(headRadius);
        head.setPosition(origin.x - headRadius, origin.y - torsoHeight / 2 - headRadius * 2);

        neck.setSize(sf::Vector2f(limbWidth, 10.f * s));
        neck.setPosition(head.getPosition().x + headRadius - limbWidth / 2, head.getPosition().y + headRadius * 2);

        // Torso
        torso.setSize(sf::Vector2f(torsoWidth, torsoHeight));
        torso.setPosition(neck.getPosition().x + limbWidth / 2 - torsoWidth / 2, neck.getPosition().y + neck.getSize().y);

        // Arms (slightly rotated outward to form a 'V' shape)
        rightArm.setSize(sf::Vector2f(limbWidth, limbLength));
        rightArm.setOrigin(limbWidth / 2, 0);
        rightArm.setPosition(torso.getPosition().x + torsoWidth, torso.getPosition().y);
        rightArm.setRotation(-15); // Rotated slightly outward

        leftArm.setSize(sf::Vector2f(limbWidth, limbLength));
        leftArm.setOrigin(limbWidth / 2, 0);
        leftArm.setPosition(torso.getPosition().x, torso.getPosition().y);
        leftArm.setRotation(15); // Rotated slightly outward

        // Hands
        rightHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        rightHand.setOrigin(handFootSize / 2, 0);
        // Position hands relative to the end of the rotated arms
        sf::FloatRect rightArmBounds = rightArm.getGlobalBounds();
        rightHand.setPosition(rightArmBounds.left + rightArmBounds.width / 2, rightArmBounds.top + rightArmBounds.height);

        leftHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        leftHand.setOrigin(handFootSize / 2, 0);
        // Position hands relative to the end of the rotated arms
        sf::FloatRect leftArmBounds = leftArm.getGlobalBounds();
        leftHand.setPosition(leftArmBounds.left + leftArmBounds.width / 2, leftArmBounds.top + leftArmBounds.height);

        // Legs (straight)
        rightLeg.setSize(sf::Vector2f(limbWidth, limbLength));
        rightLeg.setOrigin(limbWidth / 2, 0);
        rightLeg.setPosition(torso.getPosition().x + torsoWidth, torso.getPosition().y + torsoHeight);
        rightLeg.setRotation(0);

        leftLeg.setSize(sf::Vector2f(limbWidth, limbLength));
        leftLeg.setOrigin(limbWidth / 2, 0);
        leftLeg.setPosition(torso.getPosition().x, torso.getPosition().y + torsoHeight);
        leftLeg.setRotation(0);

        // Feet
        rightFoot.setSize(sf::Vector2f(handFootSize * 1.5f, handFootSize * 0.75f));
        rightFoot.setOrigin(handFootSize * 0.75f, 0);
        rightFoot.setPosition(rightLeg.getPosition().x, rightLeg.getPosition().y + limbLength);

        leftFoot.setSize(sf::Vector2f(handFootSize * 1.5f, handFootSize * 0.75f));
        leftFoot.setOrigin(handFootSize * 0.75f, 0);
        leftFoot.setPosition(leftLeg.getPosition().x, leftLeg.getPosition().y + limbLength);

        // Generate outline and interior joints from the shapes
        generateOutlineFromShapes();
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
    figure.add_skeleton_for_player(this->ID, 5.0f); // Strong skeleton for standing stability

    // Set proper mass distribution for stability
    for (auto &p : figure.points)
    {
        if (p.id == this->ID)
        {
            switch (p.body_part)
            {
            // Feet and lower legs get more mass for stability
            case BODY_PART::FOOT_L:
            case BODY_PART::FOOT_R:
                p.mass = 2.5f; // Heavy feet for grounding
                break;
            case BODY_PART::ANKLE_L:
            case BODY_PART::ANKLE_R:
                p.mass = 2.0f;
                break;
            case BODY_PART::KNEE_L:
            case BODY_PART::KNEE_R:
                p.mass = 1.5f;
                break;

            // Torso gets more mass (body weight)
            case BODY_PART::SPINE_MID:
            case BODY_PART::PELVIS:
                p.mass = 1.8f;
                break;
            case BODY_PART::SPINE_UP:
            case BODY_PART::SPINE_LOW:
                p.mass = 1.5f;
                break;

            // Head moderate mass
            case BODY_PART::HEAD:
                p.mass = 1.3f;
                break;

            // Arms lighter for mobility
            case BODY_PART::HAND_L:
            case BODY_PART::HAND_R:
            case BODY_PART::WRIST_L:
            case BODY_PART::WRIST_R:
                p.mass = 0.8f;
                break;

            // Default skeleton mass
            case BODY_PART::NONE:
                p.mass = 0.5f; // Flesh points lighter
                break;
            default:
                p.mass = 1.0f; // Standard skeleton mass
                break;
            }
        }
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

    // CRITICAL: Clear any residual part targets that might interfere with stability
    figure.partTargets.clear();
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
    // =====================================================================
    // INTEGRATED UPDATE: Stability + Control Systems Working Together
    // =====================================================================

    // 🦴 STEP 1: ALWAYS apply physiological reflexes (postural stability)
    // This is the foundation - like human unconscious balance systems
    figure.apply_postural_stability(this->ID, dt, ground_level);

    // 🎮 STEP 2: Apply intentional control system on top of stable base
    if (useAdvancedControls && bodyControlSystem)
    {
        // 🎮 ADVANCED CONTROL SYSTEM MODE
        // Apply coordinated forces that respect the stability foundation
        bodyControlSystem->update(dt);
        bodyControlSystem->applyControls(figure, ID, dt);

        // CRITICAL FIX: Bridge advanced control input to basic locomotion
        // Convert WASD from advanced controls into basic movement for locomotion
        InputManager &inputMgr = bodyControlSystem->getInputManager();

        MOVE_TYPE locomotionInput = MOVE_TYPE::NONE;

        bool left = inputMgr.isHeld(BUTTON_ACTION::MOVE_LEFT);
        bool right = inputMgr.isHeld(BUTTON_ACTION::MOVE_RIGHT);
        bool up = inputMgr.isHeld(BUTTON_ACTION::MOVE_UP);

        if (left && up)
            locomotionInput = MOVE_TYPE::UP_LEFT;
        else if (right && up)
            locomotionInput = MOVE_TYPE::UP_RIGHT;
        else if (left)
            locomotionInput = MOVE_TYPE::LEFT;
        else if (right)
            locomotionInput = MOVE_TYPE::RIGHT;
        else if (up)
            locomotionInput = MOVE_TYPE::UP;

        input = locomotionInput; // Set for simple controls to use

        // Apply basic movement forces for locomotion
        updateSimpleControls(dt);
    }
    else
    {
        // 🎯 SIMPLE CONTROL MODE (Legacy)
        updateSimpleControls(dt);
    }

    // 🔬 STEP 3: Physics integration (after all forces applied)
    updatePhysics(dt);

    // 🚶 STEP 4: Update movement animations
    if (isWalking)
        updateWalkingAnimation(dt);
    if (isWaving)
        updateWavingAnimation(dt);
}

void Player::updateSimpleControls(float dt)
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
}

void Player::updatePhysics(float dt)
{
    // NOTE: Postural stability is now applied in main update() before control systems
    // This ensures stability is the foundation, not competing with controls

    // integrate and satisfy constraints
    figure.update(dt);

    // PROPER IMPULSE-BASED GROUND COLLISION (like standard game physics)
    if (figure.points.size() > 1)
    {
        bool anyFootGrounded = false;
        const float groundY = ground_level;
        const float restitution = 0.1f; // Low bounce
        const float staticFriction = 0.8f;

        for (size_t i = 1; i < figure.points.size(); ++i)
        {
            auto &p = figure.points[i];

            // Check for penetration
            if (p.pos.y > groundY)
            {
                // POSITION CORRECTION: Project out of ground
                float penetration = p.pos.y - groundY;
                p.pos.y = groundY;

                // VELOCITY IMPULSE: Calculate relative velocity
                sf::Vector2f velocity = (p.pos - p.prev_pos) / dt;
                float relativeVelY = velocity.y; // How fast hitting ground

                if (relativeVelY > 0.1f) // Only if moving into ground
                {
                    // Calculate impulse magnitude: J = -(1 + e) * vrel
                    float impulseMagnitude = -(1.0f + restitution) * relativeVelY * p.mass;

                    // Apply impulse (change in momentum)
                    sf::Vector2f impulse = {0.f, impulseMagnitude};

                    // Convert impulse to position change for Verlet integration
                    sf::Vector2f velocityChange = impulse / p.mass;
                    p.prev_pos.y = p.pos.y - velocityChange.y * dt;

                    // FRICTION: Apply static friction to horizontal motion
                    bool isFoot = (p.body_part == BODY_PART::FOOT_L || p.body_part == BODY_PART::FOOT_R ||
                                   p.body_part == BODY_PART::ANKLE_L || p.body_part == BODY_PART::ANKLE_R);

                    if (isFoot && std::abs(velocity.x) < 50.f) // Static friction threshold
                    {
                        // Lock foot horizontally when grounded (like real physics)
                        p.prev_pos.x = p.pos.x - velocity.x * dt * (1.f - staticFriction);
                        anyFootGrounded = true;
                    }
                    else
                    {
                        // Kinetic friction for sliding
                        p.prev_pos.x = p.pos.x - velocity.x * dt * (1.f - staticFriction * 0.5f);
                    }
                }

                onGround = true;
            }
        }

        // STABILITY ENHANCEMENT: When feet are grounded, stabilize the figure
        if (anyFootGrounded)
        {
            // Find feet positions for base of support calculation
            sf::Vector2f leftFoot, rightFoot;
            bool hasLeftFoot = false, hasRightFoot = false;

            for (const auto &p : figure.points)
            {
                if (p.body_part == BODY_PART::FOOT_L)
                {
                    leftFoot = p.pos;
                    hasLeftFoot = true;
                }
                if (p.body_part == BODY_PART::FOOT_R)
                {
                    rightFoot = p.pos;
                    hasRightFoot = true;
                }
            }

            if (hasLeftFoot && hasRightFoot)
            {
                sf::Vector2f supportCenter = (leftFoot + rightFoot) * 0.5f;

                // Apply stabilizing force to keep upper body over base of support
                for (auto &p : figure.points)
                {
                    if (p.body_part == BODY_PART::HEAD || p.body_part == BODY_PART::SPINE_UP ||
                        p.body_part == BODY_PART::PELVIS)
                    {
                        float offset = p.pos.x - supportCenter.x;
                        if (std::abs(offset) > 10.f) // Only correct significant imbalance
                        {
                            // Apply restoring force proportional to offset
                            sf::Vector2f restoring = {-offset * 150.f * dt, 0.f};
                            p.acc += restoring / p.mass;
                        }
                    }
                }
            }
        }
    }
}

// 🚶 WALKING ANIMATION IMPLEMENTATION
void Player::updateWalkingAnimation(float dt)
{
    walkTimer += dt;

    // Walking cycle: alternating leg movement
    float cycle = std::sin(walkTimer * walkCycleSpeed);
    float leftLegPhase = cycle;
    float rightLegPhase = -cycle; // opposite phase

    // Get leg joint indices
    int leftHip = figure.find_part_index(ID, BODY_PART::HIP_L);
    int rightHip = figure.find_part_index(ID, BODY_PART::HIP_R);
    int leftKnee = figure.find_part_index(ID, BODY_PART::KNEE_L);
    int rightKnee = figure.find_part_index(ID, BODY_PART::KNEE_R);
    int leftFoot = figure.find_part_index(ID, BODY_PART::FOOT_L);
    int rightFoot = figure.find_part_index(ID, BODY_PART::FOOT_R);
    int pelvis = figure.find_part_index(ID, BODY_PART::PELVIS);

    if (leftHip > 0 && rightHip > 0 && leftKnee > 0 && rightKnee > 0 && leftFoot > 0 && rightFoot > 0 && pelvis > 0)
    {
        // CRITICAL FIX: Apply walking momentum like jumping (instantaneous impulse)
        // Use the SAME technique that makes jumping work successfully
        sf::Vector2f walkingImpulse = sf::Vector2f(walkDirection * walkForwardSpeed, 0.0f);

        // Apply to center of mass (points[0]) with dt=1.0f for instantaneous impulse, like jumping
        if (figure.points.size() > 0)
        {
            figure.apply_force(walkingImpulse, figure.points[0].pos, figure.get_radius() * 1.5f, 1.0f);
        }

        // Get current positions
        sf::Vector2f leftHipPos = figure.points[leftHip].pos;
        sf::Vector2f rightHipPos = figure.points[rightHip].pos;

        // Walking step parameters
        float stepHeight = 20.0f;
        float stepLength = 30.0f;

        // Left leg movement (adjusted for forward walking)
        sf::Vector2f leftFootTarget = leftHipPos + sf::Vector2f(
                                                       stepLength * leftLegPhase * 0.3f + walkDirection * 10.0f,
                                                       60.0f + stepHeight * std::max(0.0f, leftLegPhase) // lift foot when moving forward
                                                   );

        // Right leg movement (adjusted for forward walking)
        sf::Vector2f rightFootTarget = rightHipPos + sf::Vector2f(
                                                         stepLength * rightLegPhase * 0.3f + walkDirection * 10.0f,
                                                         60.0f + stepHeight * std::max(0.0f, rightLegPhase) // lift foot when moving forward
                                                     );

        // Apply gentle walking targets (work WITH stability system)
        figure.set_part_target(ID, BODY_PART::FOOT_L, leftFootTarget, 150.0f, 20.0f);  // Increased strength
        figure.set_part_target(ID, BODY_PART::FOOT_R, rightFootTarget, 150.0f, 20.0f); // Increased strength

        // Slight knee bend during walk cycle
        figure.animate_skeleton_spring(ID, BODY_PART::HIP_L, BODY_PART::KNEE_L,
                                       45.0f + 10.0f * std::abs(leftLegPhase), 0.3f);
        figure.animate_skeleton_spring(ID, BODY_PART::HIP_R, BODY_PART::KNEE_R,
                                       45.0f + 10.0f * std::abs(rightLegPhase), 0.3f);

        // Add subtle body sway for natural walking
        if (pelvis > 0)
        {
            sf::Vector2f sway = sf::Vector2f(std::sin(walkTimer * walkCycleSpeed * 2.0f) * 5.0f, 0.0f);
            sf::Vector2f pelvisTarget = figure.points[pelvis].pos + sway;
            figure.set_part_target(ID, BODY_PART::PELVIS, pelvisTarget, 50.0f, 10.0f); // Gentle sway
        }
    }
}

// 👋 WAVING ANIMATION IMPLEMENTATION
void Player::updateWavingAnimation(float dt)
{
    waveTimer += dt;

    // Find hand and shoulder
    int rightHand = figure.find_part_index(ID, BODY_PART::HAND_R);
    int rightShoulder = figure.find_part_index(ID, BODY_PART::SHO_R);
    int rightElbow = figure.find_part_index(ID, BODY_PART::ELB_R);
    int rightWrist = figure.find_part_index(ID, BODY_PART::WRIST_R);

    if (rightHand > 0 && rightShoulder > 0)
    {
        sf::Vector2f shoulderPos = figure.points[rightShoulder].pos;

        // Wave motion: circular movement
        float freq = 6.0f;       // Hz
        float amplitude = 40.0f; // Increased amplitude for more visible movement
        float angle = waveTimer * freq;
        sf::Vector2f offset = sf::Vector2f(std::cos(angle) * amplitude, std::sin(angle) * amplitude * 0.6f);

        // Position hand relative to shoulder, not constrained to torso
        sf::Vector2f baseHandPos = shoulderPos + sf::Vector2f(70.0f, -40.0f); // Further out from shoulder
        sf::Vector2f handTarget = baseHandPos + offset;

        // CRITICAL FIX: Use much gentler forces - the 800.0f was causing the player to fly!
        // Apply force directly to the hand particle but with reasonable magnitude
        sf::Vector2f handCurrentPos = figure.points[rightHand].pos;
        sf::Vector2f handForce = (handTarget - handCurrentPos) * 50.0f; // Much gentler force

        // Apply as impulse (dt = 1.0f) to overcome constraints but not launch the player
        figure.apply_force(handForce, handCurrentPos, 15.0f, 1.0f);

        // Apply progressive forces down the arm chain with gentler magnitudes
        if (rightWrist > 0)
        {
            sf::Vector2f wristTarget = shoulderPos + (handTarget - shoulderPos) * 0.75f;
            sf::Vector2f wristCurrentPos = figure.points[rightWrist].pos;
            sf::Vector2f wristForce = (wristTarget - wristCurrentPos) * 40.0f; // Reduced from 600.0f
            figure.apply_force(wristForce, wristCurrentPos, 12.0f, 1.0f);
        }

        if (rightElbow > 0)
        {
            sf::Vector2f elbowTarget = shoulderPos + (handTarget - shoulderPos) * 0.5f;
            sf::Vector2f elbowCurrentPos = figure.points[rightElbow].pos;
            sf::Vector2f elbowForce = (elbowTarget - elbowCurrentPos) * 30.0f; // Reduced from 400.0f
            figure.apply_force(elbowForce, elbowCurrentPos, 10.0f, 1.0f);
        }

        // Dramatically weaken skeleton constraints during waving to allow free movement
        // Reduce arm skeleton spring strength to allow natural arm movement
        figure.animate_skeleton_spring(ID, BODY_PART::SHO_R, BODY_PART::ELB_R, 40.0f, 0.1f);    // Very weak
        figure.animate_skeleton_spring(ID, BODY_PART::ELB_R, BODY_PART::WRIST_R, 30.0f, 0.1f);  // Very weak
        figure.animate_skeleton_spring(ID, BODY_PART::WRIST_R, BODY_PART::HAND_R, 20.0f, 0.1f); // Very weak
    }

    // Stop waving after duration
    if (waveTimer > waveDuration)
    {
        stopWaving();
    }
}

// Clear animation targets
void Player::clearWalkTargets()
{
    figure.clear_part_target(ID, BODY_PART::FOOT_L);
    figure.clear_part_target(ID, BODY_PART::FOOT_R);
    figure.clear_part_target(ID, BODY_PART::KNEE_L);
    figure.clear_part_target(ID, BODY_PART::KNEE_R);
}

void Player::clearWaveTargets()
{
    figure.clear_part_target(ID, BODY_PART::HAND_R);
    figure.clear_part_target(ID, BODY_PART::ELB_R);
}
