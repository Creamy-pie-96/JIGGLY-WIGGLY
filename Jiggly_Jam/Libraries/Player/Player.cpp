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

    std::vector<sf::Vector2f> sampleRectangle(const sf::RectangleShape &rect, int pointsPerSide) const
    {
        std::vector<sf::Vector2f> points;
        sf::Vector2f pos = rect.getPosition();
        sf::Vector2f size = rect.getSize();
        sf::Vector2f origin = rect.getOrigin();
        float rotation = rect.getRotation() * M_PI / 180.f;

        std::vector<sf::Vector2f> corners = {
            {-origin.x, -origin.y},
            {size.x - origin.x, -origin.y},
            {size.x - origin.x, size.y - origin.y},
            {-origin.x, size.y - origin.y}
        };

        for (auto &corner : corners)
        {
            float x = corner.x * std::cos(rotation) - corner.y * std::sin(rotation);
            float y = corner.x * std::sin(rotation) + corner.y * std::cos(rotation);
            corner = sf::Vector2f(x + pos.x, y + pos.y);
        }

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

    BODY_PART classifyPoint(const sf::Vector2f &point) const
    {
        struct JointInfo
        {
            sf::Vector2f pos;
            BODY_PART part;
            float radius;
        };

        std::vector<JointInfo> joints = {
            {head.getPosition() + sf::Vector2f(head.getRadius(), head.getRadius()), BODY_PART::HEAD, head.getRadius() * 1.f}, // Increased radius for head
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
            {rightFoot.getPosition() + rightFoot.getSize() * 0.5f, BODY_PART::FOOT_R, 10.f * s},
            {leftFoot.getPosition() + leftFoot.getSize() * 0.5f, BODY_PART::FOOT_L, 10.f * s}};

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

    void generateOutlineFromShapes()
    {
        std::vector<sf::Vector2f> allPoints;

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

        sf::Vector2f center{0.f, 0.f};
        for (const auto &p : allPoints)
            center += p;
        center /= (float)allPoints.size();

        std::sort(allPoints.begin(), allPoints.end(), [center](const sf::Vector2f &a, const sf::Vector2f &b)
                  {
            float angleA = std::atan2(a.y - center.y, a.x - center.x);
            float angleB = std::atan2(b.y - center.y, b.x - center.x);
            return angleA < angleB; });

        outline.clear();
        for (const auto &point : allPoints)
        {
            BODY_PART part = classifyPoint(point);
            outline.push_back({point, part});
        }

        generateInteriorJoints();
    }

    void generateInteriorJoints()
    {
        sf::Vector2f torsoCenter = torso.getPosition() + torso.getSize() * 0.5f;
        sf::Vector2f neckBottom = neck.getPosition() + sf::Vector2f(neck.getSize().x * 0.5f, neck.getSize().y);

        interior_joints = {
            {sf::Vector2f(torsoCenter.x, neckBottom.y + torso.getSize().y * 0.2f), BODY_PART::SPINE_UP},
            {torsoCenter, BODY_PART::SPINE_MID},
            {sf::Vector2f(torsoCenter.x, torsoCenter.y + torso.getSize().y * 0.3f), BODY_PART::SPINE_LOW},
            {sf::Vector2f(torsoCenter.x, torso.getPosition().y + torso.getSize().y), BODY_PART::PELVIS}};
    }

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

        float headRadius = 15.f * s;
        float limbWidth = 8.f * s;
        float torsoWidth = 25.f * s;
        float torsoHeight = 50.f * s;
        float limbLength = 50.f * s;
        float handFootSize = 10.f * s;

        head.setRadius(headRadius);
        head.setPosition(origin.x - headRadius, origin.y - torsoHeight / 2 - headRadius * 2);

        neck.setSize(sf::Vector2f(limbWidth, 10.f * s));
        neck.setPosition(head.getPosition().x + headRadius - limbWidth / 2, head.getPosition().y + headRadius * 2);

        torso.setSize(sf::Vector2f(torsoWidth, torsoHeight));
        torso.setPosition(neck.getPosition().x + limbWidth / 2 - torsoWidth / 2, neck.getPosition().y + neck.getSize().y);

        rightArm.setSize(sf::Vector2f(limbWidth, limbLength));
        rightArm.setOrigin(limbWidth / 2, 0);
        rightArm.setPosition(torso.getPosition().x + torsoWidth, torso.getPosition().y);
        rightArm.setRotation(-15);

        leftArm.setSize(sf::Vector2f(limbWidth, limbLength));
        leftArm.setOrigin(limbWidth / 2, 0);
        leftArm.setPosition(torso.getPosition().x, torso.getPosition().y);
        leftArm.setRotation(15);

        rightHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        rightHand.setOrigin(handFootSize / 2, 0);
        sf::FloatRect rightArmBounds = rightArm.getGlobalBounds();
        rightHand.setPosition(rightArmBounds.left + rightArmBounds.width / 2, rightArmBounds.top + rightArmBounds.height);

        leftHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        leftHand.setOrigin(handFootSize / 2, 0);
        sf::FloatRect leftArmBounds = leftArm.getGlobalBounds();
        leftHand.setPosition(leftArmBounds.left + leftArmBounds.width / 2, leftArmBounds.top + leftArmBounds.height);

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

        // DEBUG: Print key positions for debugging
        sf::Vector2f headCenter = head.getPosition() + sf::Vector2f(head.getRadius(), head.getRadius());
        sf::Vector2f leftFootCenter = leftFoot.getPosition() + leftFoot.getSize() * 0.5f;
        sf::Vector2f rightFootCenter = rightFoot.getPosition() + rightFoot.getSize() * 0.5f;

        std::cout << "DEBUG SHAPE POSITIONS:" << std::endl;
        std::cout << "  HEAD center: (" << headCenter.x << ", " << headCenter.y << ") radius: " << head.getRadius() * 1.2f << std::endl;
        std::cout << "  LEFT_FOOT center: (" << leftFootCenter.x << ", " << leftFootCenter.y << ") radius: " << 10.f * s << std::endl;
        std::cout << "  RIGHT_FOOT center: (" << rightFootCenter.x << ", " << rightFootCenter.y << ") radius: " << 10.f * s << std::endl;
        std::cout << "  Scale factor s: " << s << std::endl;

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

    figure.add_skeleton_for_player(this->ID, 5.0f); // Strong skeleton for standing stability

    for (auto &p : figure.points)
    {
        if (p.id == this->ID)
        {
            switch (p.body_part)
            {
            // Feet and lower legs get much more mass for stability and grounding
            case BODY_PART::FOOT_L:
            case BODY_PART::FOOT_R:
                p.mass = 3.5f; // Much heavier feet for better grounding and stability
                break;
            case BODY_PART::ANKLE_L:
            case BODY_PART::ANKLE_R:
                p.mass = 2.8f; // Heavy ankles for low center of mass
                break;
            case BODY_PART::KNEE_L:
            case BODY_PART::KNEE_R:
                p.mass = 2.2f; // Heavy knees for leg stability
                break;
            case BODY_PART::HIP_L:
            case BODY_PART::HIP_R:
                p.mass = 2.0f; // Heavy hips for core stability
                break;

            // Lower torso heavy, upper torso lighter
            case BODY_PART::PELVIS:
                p.mass = 2.5f; // Heavy pelvis for low center of mass
                break;
            case BODY_PART::SPINE_LOW:
                p.mass = 2.0f; // Heavy lower spine
                break;
            case BODY_PART::SPINE_MID:
                p.mass = 1.5f; // Moderate middle spine
                break;
            case BODY_PART::SPINE_UP:
                p.mass = 1.2f; // Lighter upper spine
                break;

            // Head much lighter to reduce top-heavy instability
            case BODY_PART::HEAD:
                p.mass = 0.8f; // Much lighter head to reduce torque and instability
                break;
            case BODY_PART::NECK:
                p.mass = 0.9f; // Light neck
                break;

            // Arms and shoulders lighter for mobility
            case BODY_PART::CLAV_L:
            case BODY_PART::CLAV_R:
                p.mass = 0.9f;
                break;
            case BODY_PART::SHO_L:
            case BODY_PART::SHO_R:
                p.mass = 1.0f;
                break;
            case BODY_PART::ELB_L:
            case BODY_PART::ELB_R:
                p.mass = 0.9f;
                break;
            case BODY_PART::WRIST_L:
            case BODY_PART::WRIST_R:
                p.mass = 0.7f;
                break;
            case BODY_PART::HAND_L:
            case BODY_PART::HAND_R:
                p.mass = 0.6f; // Very light hands
                break;

            // Flesh points very light
            case BODY_PART::NONE:
                p.mass = 0.3f; // Lighter flesh points
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

    // STABILITY FIX: Enforce upright pose immediately after spawn
    for (auto &p : figure.points)
    {
        if (p.id == this->ID && p.body_part != BODY_PART::NONE)
        {
            p.prev_pos = p.pos; // Zero velocity
        }
    }

    int pelvis_idx = figure.find_part_index(ID, BODY_PART::PELVIS);
    int spine_low_idx = figure.find_part_index(ID, BODY_PART::SPINE_LOW);
    int spine_mid_idx = figure.find_part_index(ID, BODY_PART::SPINE_MID);
    int spine_up_idx = figure.find_part_index(ID, BODY_PART::SPINE_UP);
    int neck_idx = figure.find_part_index(ID, BODY_PART::NECK);
    int head_idx = figure.find_part_index(ID, BODY_PART::HEAD);

    // Align spine vertically from pelvis up
    if (pelvis_idx >= 0 && head_idx >= 0)
    {
        sf::Vector2f pelvis_pos = figure.points[pelvis_idx].pos;

        if (spine_low_idx >= 0)
        {
            figure.points[spine_low_idx].pos = pelvis_pos + sf::Vector2f(0, -15);
            figure.points[spine_low_idx].prev_pos = figure.points[spine_low_idx].pos;
        }
        if (spine_mid_idx >= 0)
        {
            figure.points[spine_mid_idx].pos = pelvis_pos + sf::Vector2f(0, -30);
            figure.points[spine_mid_idx].prev_pos = figure.points[spine_mid_idx].pos;
        }
        if (spine_up_idx >= 0)
        {
            figure.points[spine_up_idx].pos = pelvis_pos + sf::Vector2f(0, -45);
            figure.points[spine_up_idx].prev_pos = figure.points[spine_up_idx].pos;
        }
        if (neck_idx >= 0)
        {
            figure.points[neck_idx].pos = pelvis_pos + sf::Vector2f(0, -60);
            figure.points[neck_idx].prev_pos = figure.points[neck_idx].pos;
        }
        if (head_idx >= 0)
        {
            figure.points[head_idx].pos = pelvis_pos + sf::Vector2f(0, -75);
            figure.points[head_idx].prev_pos = figure.points[head_idx].pos;
        }
    }

    // CRITICAL: Ensure feet are properly positioned for stability
    int foot_l_idx = figure.find_part_index(ID, BODY_PART::FOOT_L);
    int foot_r_idx = figure.find_part_index(ID, BODY_PART::FOOT_R);
    int ankle_l_idx = figure.find_part_index(ID, BODY_PART::ANKLE_L);
    int ankle_r_idx = figure.find_part_index(ID, BODY_PART::ANKLE_R);

    if (pelvis_idx >= 0)
    {
        sf::Vector2f pelvis_pos = figure.points[pelvis_idx].pos;

        if (foot_l_idx >= 0)
        {
            figure.points[foot_l_idx].pos = pelvis_pos + sf::Vector2f(-20, 80);
            figure.points[foot_l_idx].prev_pos = figure.points[foot_l_idx].pos;
        }
        if (foot_r_idx >= 0)
        {
            figure.points[foot_r_idx].pos = pelvis_pos + sf::Vector2f(20, 80);
            figure.points[foot_r_idx].prev_pos = figure.points[foot_r_idx].pos;
        }
        if (ankle_l_idx >= 0)
        {
            figure.points[ankle_l_idx].pos = pelvis_pos + sf::Vector2f(-18, 65);
            figure.points[ankle_l_idx].prev_pos = figure.points[ankle_l_idx].pos;
        }
        if (ankle_r_idx >= 0)
        {
            figure.points[ankle_r_idx].pos = pelvis_pos + sf::Vector2f(18, 65);
            figure.points[ankle_r_idx].prev_pos = figure.points[ankle_r_idx].pos;
        }
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
        // SIMPLIFIED JUMPING: Apply force only to center of mass for natural motion
        sf::Vector2f jumpForce = sf::Vector2f(0.f, -jump_force);

        if (figure.points.size() > 0)
        {
            // This prevents the twisted/bent shapes caused by conflicting forces
            figure.apply_force(jumpForce, figure.points[0].pos, figure.get_radius() * 1.5f, 1.f);
        }

        onGround = false;
        coyoteTimer = 0.f;

        // No jump targets needed - let natural physics handle the motion
    }
}

void Player::update(float dt)
{
    // ADVANCED PHYSICS-DRIVEN WALKING CYCLE SYSTEM
    // Complete replacement with sophisticated physics integration

    // STEP 1: Apply gravitational forces globally
    // The Jelly physics engine automatically handles gravity in update_verlet,
    // but we can apply additional global forces if needed for gameplay
    if (figure.points.size() > 0)
    {
        // figure.apply_force(sf::Vector2f(0.0f, customGravityForce), figure.points[0].pos, figure.get_radius() * 2.0f);

        // sf::Vector2f environmentalForce = calculateEnvironmentalForces();
        // figure.apply_force(environmentalForce, figure.points[0].pos, figure.get_radius() * 2.0f);
    }

    // STEP 2: Update ground contact detection and response
    // This handles foot-ground interactions and collision response for all body parts
    figure.updateGroundContact(this->ID, dt, ground_level);

    // STEP 3: Determine movement input and execute physics-driven stepping
    bool leftPressed = false;
    bool rightPressed = false;
    bool upPressed = false;

    // Extract movement input from control systems
    if (useAdvancedControls && bodyControlSystem)
    {
        // Advanced control system input handling
        InputManager &inputMgr = bodyControlSystem->getInputManager();
        leftPressed = inputMgr.isHeld(BUTTON_ACTION::MOVE_LEFT);
        rightPressed = inputMgr.isHeld(BUTTON_ACTION::MOVE_RIGHT);
        upPressed = inputMgr.isHeld(BUTTON_ACTION::MOVE_UP);

        bodyControlSystem->update(dt);
        bodyControlSystem->applyControls(figure, ID, dt);
    }
    else
    {
        // Simple control system input handling
        leftPressed = (input == MOVE_TYPE::LEFT || input == MOVE_TYPE::UP_LEFT);
        rightPressed = (input == MOVE_TYPE::RIGHT || input == MOVE_TYPE::UP_RIGHT);
        upPressed = (input == MOVE_TYPE::UP || input == MOVE_TYPE::UP_LEFT || input == MOVE_TYPE::UP_RIGHT);
    }

    // STEP 4: Execute physics-driven stepping when on ground
    bool isOnGround = this->is_onGround();

    if (isOnGround && (leftPressed || rightPressed))
    {
        // Determine stepping pattern - alternate feet for realistic walking
        static float stepTimer = 0.0f;
        static bool useLeftFoot = true;
        static int lastMoveDirection = 0; // -1 left, 0 none, 1 right

        stepTimer += dt;

        // Determine current movement direction
        int currentMoveDirection = 0;
        if (leftPressed && !rightPressed)
            currentMoveDirection = -1;
        else if (rightPressed && !leftPressed)
            currentMoveDirection = 1;

        // Switch stepping foot based on movement direction change or time
        bool directionChanged = (currentMoveDirection != lastMoveDirection && currentMoveDirection != 0);
        bool timeToSwitch = stepTimer > 0.6f; // Switch feet every 0.6 seconds

        if (directionChanged || timeToSwitch)
        {
            useLeftFoot = !useLeftFoot;
            stepTimer = 0.0f;
        }

        lastMoveDirection = currentMoveDirection;

        // Execute physics step with the appropriate foot
        BODY_PART stepping_foot = useLeftFoot ? BODY_PART::FOOT_L : BODY_PART::FOOT_R;
        figure.executePhysicsStep(this->ID, dt, stepping_foot);

        // This works in conjunction with the stepping system
        sf::Vector2f lateral_force(0.0f, 0.0f);
        float forceMultiplier = 1.2f; // ENHANCED: Increased from 0.8f for better movement responsiveness

        if (leftPressed && rightPressed)
        {
            // Both directions - minimal force, let physics stepping dominate
            lateral_force = sf::Vector2f(0.0f, 0.0f);
        }
        else if (leftPressed)
        {
            lateral_force = sf::Vector2f(-move_force * forceMultiplier, 0.0f);
        }
        else if (rightPressed)
        {
            lateral_force = sf::Vector2f(move_force * forceMultiplier, 0.0f);
        }

        if (figure.points.size() > 0 && (lateral_force.x != 0.0f || lateral_force.y != 0.0f))
        {
            figure.apply_force(lateral_force, figure.points[0].pos, figure.get_radius() * 1.5f, dt);
        }

        if (!isWalking)
        {
            startWalking();
        }

        if (leftPressed && !rightPressed)
        {
            setWalkDirection(-1.0f);
        }
        else if (rightPressed && !leftPressed)
        {
            setWalkDirection(1.0f);
        }
    }
    else
    {
        // No movement input - stop walking animation
        if (isWalking)
        {
            stopWalking();
        }

        if (!isOnGround || (!leftPressed && !rightPressed))
        {
            float dampingFactor = std::exp(-air_drag * dt);
            for (auto &p : figure.points)
            {
                if (p.locked)
                    continue;
                sf::Vector2f vel = p.pos - p.prev_pos;
                vel.x *= dampingFactor;
                p.prev_pos = p.pos - vel;
            }
        }
    }

    // STEP 5: Handle jumping with physics-driven mechanics
    if (upPressed && (isOnGround || coyoteTimer > 0.f))
    {
        request_jump();
    }

    // STEP 6: Apply dynamic balancing to maintain character stability
    // This is the core system that keeps the character upright and stable
    // figure.applyDynamicBalancing(this->ID, dt); // Disabled for stability testing

    // This provides the unconscious balance reflexes that make the character feel natural
    // {
    //     // Use Gang Beasts enhanced stability with reaction delays and overcompensation
    //     figure.apply_enhanced_postural_stability(this->ID, dt, ground_level);
    // }
    // else
    // {
    // Fall back to original postural stability system
    figure.apply_postural_stability(this->ID, dt, ground_level);
    // }

    // STEP 8: Update movement animations that complement physics-driven motion
    if (isWalking)
    {
        updateWalkingAnimation(dt);
    }
    if (isWaving)
    {
        updateWavingAnimation(dt);
    }

    // STEP 9: Update state tracking and timers
    if (coyoteTimer > 0.f)
    {
        coyoteTimer -= dt;
    }

    bool newGroundState = false;
    if (figure.points.size() > 0)
    {
        for (const auto &p : figure.points)
        {
            if (p.id == this->ID &&
                (p.body_part == BODY_PART::FOOT_L || p.body_part == BODY_PART::FOOT_R ||
                 p.body_part == BODY_PART::ANKLE_L || p.body_part == BODY_PART::ANKLE_R))
            {
                float groundThreshold = 5.0f; // FIXED: More accurate ground detection
                if (p.pos.y >= ground_level - groundThreshold)
                {
                    newGroundState = true;
                    break;
                }
            }
        }
    }

    // Smooth ground state transitions
    if (newGroundState && !onGround)
    {
        // Just landed
        set_onGround(true);
    }
    else if (!newGroundState && onGround)
    {
        // Just left ground
        onGround = false;
        coyoteTimer = 0.12f; // Grace period for jumping
    }

    // STEP 10: Advanced physics integration - final step
    // This integrates all applied forces and resolves constraints
    updatePhysics(dt);

    // STEP 11: Bridge to other systems for compatibility
    // Convert current movement state to simple input for other systems
    if (useAdvancedControls && bodyControlSystem)
    {
        MOVE_TYPE locomotionInput = MOVE_TYPE::NONE;

        if (leftPressed && upPressed)
            locomotionInput = MOVE_TYPE::UP_LEFT;
        else if (rightPressed && upPressed)
            locomotionInput = MOVE_TYPE::UP_RIGHT;
        else if (leftPressed)
            locomotionInput = MOVE_TYPE::LEFT;
        else if (rightPressed)
            locomotionInput = MOVE_TYPE::RIGHT;
        else if (upPressed)
            locomotionInput = MOVE_TYPE::UP;

        input = locomotionInput; // Update for other systems
    }

    int current_move_sign = 0;
    if (leftPressed && !rightPressed)
        current_move_sign = -1;
    else if (rightPressed && !leftPressed)
        current_move_sign = 1;

    last_move_sign = current_move_sign;
    last_input = input;
}

void Player::updateSimpleControls(float dt)
{
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

    if (figure.points.size() > 1)
    {
        bool anyFootGrounded = false;
        const float groundY = ground_level;
        const float restitution = 0.1f; // Low bounce
        const float staticFriction = 0.8f;

        for (size_t i = 1; i < figure.points.size(); ++i)
        {
            auto &p = figure.points[i];

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
                    float impulseMagnitude = -(1.0f + restitution) * relativeVelY * p.mass;

                    sf::Vector2f impulse = {0.f, impulseMagnitude};

                    // Convert impulse to position change for Verlet integration
                    sf::Vector2f velocityChange = impulse / p.mass;
                    p.prev_pos.y = p.pos.y - velocityChange.y * dt;

                    // ENHANCED FRICTION: Apply comprehensive friction system
                    bool isFoot = (p.body_part == BODY_PART::FOOT_L || p.body_part == BODY_PART::FOOT_R ||
                                   p.body_part == BODY_PART::ANKLE_L || p.body_part == BODY_PART::ANKLE_R);

                    if (isFoot)
                    {
                        // Strong friction for feet to prevent sliding
                        if (std::abs(velocity.x) < 50.f) // Static friction threshold
                        {
                            // High static friction - almost lock feet when grounded
                            p.prev_pos.x = p.pos.x - velocity.x * dt * (1.f - staticFriction * 1.2f);
                            anyFootGrounded = true;
                        }
                        else
                        {
                            // Kinetic friction for sliding feet
                            p.prev_pos.x = p.pos.x - velocity.x * dt * (1.f - staticFriction * 0.7f);
                        }
                    }
                    else
                    {
                        // Moderate friction for other body parts touching ground
                        p.prev_pos.x = p.pos.x - velocity.x * dt * (1.f - staticFriction * 0.4f);
                    }
                }

                onGround = true;
            }
        }

        // STABILITY ENHANCEMENT: When feet are grounded, stabilize the figure
        if (anyFootGrounded)
        {
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

                for (auto &p : figure.points)
                {
                    if (p.body_part == BODY_PART::HEAD || p.body_part == BODY_PART::SPINE_UP ||
                        p.body_part == BODY_PART::PELVIS)
                    {
                        float offset = p.pos.x - supportCenter.x;
                        if (std::abs(offset) > 10.f) // Only correct significant imbalance
                        {
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

    int leftHip = figure.find_part_index(ID, BODY_PART::HIP_L);
    int rightHip = figure.find_part_index(ID, BODY_PART::HIP_R);
    int leftKnee = figure.find_part_index(ID, BODY_PART::KNEE_L);
    int rightKnee = figure.find_part_index(ID, BODY_PART::KNEE_R);
    int leftFoot = figure.find_part_index(ID, BODY_PART::FOOT_L);
    int rightFoot = figure.find_part_index(ID, BODY_PART::FOOT_R);
    int pelvis = figure.find_part_index(ID, BODY_PART::PELVIS);

    if (leftHip > 0 && rightHip > 0 && leftKnee > 0 && rightKnee > 0 && leftFoot > 0 && rightFoot > 0 && pelvis > 0)
    {
        // initial forward momentum. Previously we applied an instantaneous impulse
        // every frame which accumulated and caused large overshoot when holding
        // movement keys. Keep jump behavior unchanged (jump impulses are event-driven).
        if (figure.points.size() > 0)
        {
            // If we're on the first frame of the walking cycle, apply a reduced impulse.
            // walkTimer is reset to 0.0f in startWalking(), so use a small window of dt
            // to detect the first frame.
            if (walkTimer <= dt * 1.5f)
            {
                float impulseScale = 0.25f; // ENHANCED: Increased from 0.15f for better movement
                sf::Vector2f walkingImpulse = sf::Vector2f(walkDirection * walkForwardSpeed * impulseScale, 0.0f);
                figure.apply_force(walkingImpulse, figure.points[0].pos, figure.get_radius() * 1.5f, 1.0f);
            }
            else
            {
                float continuousScale = 0.15f; // ENHANCED: Increased from 0.08f for better movement
                sf::Vector2f continuousForce = sf::Vector2f(walkDirection * walkForwardSpeed * continuousScale * dt, 0.0f);
                figure.apply_force(continuousForce, figure.points[0].pos, figure.get_radius() * 1.5f);
            }
        }

        sf::Vector2f leftHipPos = figure.points[leftHip].pos;
        sf::Vector2f rightHipPos = figure.points[rightHip].pos;

        // Walking step parameters - INCREASED for better movement
        float stepHeight = 35.0f; // Increased from 20.0f
        float stepLength = 50.0f; // Increased from 30.0f

        // ENHANCED: Apply forces to entire leg chain for natural walking
        // Left leg movement - full chain coordination
        sf::Vector2f leftHipTarget = leftHipPos + sf::Vector2f(
                                                      stepLength * leftLegPhase * 0.2f + walkDirection * 8.0f,
                                                      -5.0f + stepHeight * 0.3f * std::max(0.0f, leftLegPhase));

        sf::Vector2f leftKneeTarget = leftHipPos + sf::Vector2f(
                                                       stepLength * leftLegPhase * 0.4f + walkDirection * 12.0f,
                                                       30.0f + stepHeight * 0.6f * std::max(0.0f, leftLegPhase));

        sf::Vector2f leftFootTarget = leftHipPos + sf::Vector2f(
                                                       stepLength * leftLegPhase * 0.5f + walkDirection * 15.0f,
                                                       60.0f + stepHeight * std::max(0.0f, leftLegPhase));

        // Right leg movement - full chain coordination
        sf::Vector2f rightHipTarget = rightHipPos + sf::Vector2f(
                                                        stepLength * rightLegPhase * 0.2f + walkDirection * 8.0f,
                                                        -5.0f + stepHeight * 0.3f * std::max(0.0f, rightLegPhase));

        sf::Vector2f rightKneeTarget = rightHipPos + sf::Vector2f(
                                                         stepLength * rightLegPhase * 0.4f + walkDirection * 12.0f,
                                                         30.0f + stepHeight * 0.6f * std::max(0.0f, rightLegPhase));

        sf::Vector2f rightFootTarget = rightHipPos + sf::Vector2f(
                                                         stepLength * rightLegPhase * 0.5f + walkDirection * 15.0f,
                                                         60.0f + stepHeight * std::max(0.0f, rightLegPhase));

        float targetKp = 40.0f; // Increased from 25.0f
        float targetKd = 8.0f;  // Increased from 3.0f

        figure.set_part_target(ID, BODY_PART::HIP_L, leftHipTarget, targetKp * 0.5f, targetKd * 0.5f);
        figure.set_part_target(ID, BODY_PART::HIP_R, rightHipTarget, targetKp * 0.5f, targetKd * 0.5f);
        figure.set_part_target(ID, BODY_PART::KNEE_L, leftKneeTarget, targetKp * 0.7f, targetKd * 0.7f);
        figure.set_part_target(ID, BODY_PART::KNEE_R, rightKneeTarget, targetKp * 0.7f, targetKd * 0.7f);
        figure.set_part_target(ID, BODY_PART::FOOT_L, leftFootTarget, targetKp, targetKd);
        figure.set_part_target(ID, BODY_PART::FOOT_R, rightFootTarget, targetKp, targetKd);

        // Slight knee bend during walk cycle
        figure.animate_skeleton_spring(ID, BODY_PART::HIP_L, BODY_PART::KNEE_L,
                                       45.0f + 10.0f * std::abs(leftLegPhase), 0.3f);
        figure.animate_skeleton_spring(ID, BODY_PART::HIP_R, BODY_PART::KNEE_R,
                                       45.0f + 10.0f * std::abs(rightLegPhase), 0.3f);

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

        sf::Vector2f baseHandPos = shoulderPos + sf::Vector2f(70.0f, -40.0f); // Further out from shoulder
        sf::Vector2f handTarget = baseHandPos + offset;

        // PHYSICS FIX: Much gentler waving forces to prevent flight
        sf::Vector2f handCurrentPos = figure.points[rightHand].pos;
        sf::Vector2f handForce = (handTarget - handCurrentPos) * 8.0f; // MAJOR FIX: Reduced from 50.0f

        figure.apply_force(handForce, handCurrentPos, 15.0f, 1.0f);

        if (rightWrist > 0)
        {
            sf::Vector2f wristTarget = shoulderPos + (handTarget - shoulderPos) * 0.75f;
            sf::Vector2f wristCurrentPos = figure.points[rightWrist].pos;
            sf::Vector2f wristForce = (wristTarget - wristCurrentPos) * 6.0f; // MAJOR FIX: Reduced from 40.0f
            figure.apply_force(wristForce, wristCurrentPos, 12.0f, 1.0f);
        }

        if (rightElbow > 0)
        {
            sf::Vector2f elbowTarget = shoulderPos + (handTarget - shoulderPos) * 0.5f;
            sf::Vector2f elbowCurrentPos = figure.points[rightElbow].pos;
            sf::Vector2f elbowForce = (elbowTarget - elbowCurrentPos) * 4.0f; // MAJOR FIX: Reduced from 30.0f
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


bool Player::loadGangBeastsSettings(const std::string &settingsPath)
{
    if (!settingsParser)
    {
        std::cout << "⚠️ Settings parser not initialized!" << std::endl;
        return false;
    }

    bool success = settingsParser->loadSettings(settingsPath);

    // Always apply settings (loaded or defaults) to the Jelly so it has a valid
    // pointer to physics tuning data. Previously we only applied settings when
    // loadSettings returned true, which left the Jelly with a null pointer when
    // the file was missing (defaults were still applied in the parser), causing
    // runtime null-dereferences in walking code.
    if (gangBeastsPhysicsEnabled)
    {
        const auto &settings = settingsParser->getSettings();
        figure.setGangBeastsSettings(&settings);

        std::cout << "🎮 Gang Beasts physics enabled for Player " << ID << std::endl;
        std::cout << "⚙️ Skeleton stiffness: " << (settings.physics_personality.skeleton_springs.base_stiffness_multiplier * 100) << "%" << std::endl;
        std::cout << "⚖️ Stability delay: " << (settings.postural_stability.reaction_timing.base_reaction_delay * 1000) << "ms" << std::endl;
    }

    return success;
}

const GangBeastsSettings::GangBeastsPhysicsSettings *Player::getGangBeastsSettings() const
{
    if (!settingsParser || !settingsParser->isLoaded())
    {
        return nullptr;
    }
    return &settingsParser->getSettings();
}
