#include "BodyControlSystem.hpp"
#include <algorithm>
#include <iostream>

std::string getBodyPartName(BODY_PART part)
{
    switch (part)
    {
    case BODY_PART::HEAD:
        return "HEAD";
    case BODY_PART::PELVIS:
        return "PELVIS";
    case BODY_PART::SHO_L:
        return "SHO_L";
    case BODY_PART::SHO_R:
        return "SHO_R";
    case BODY_PART::ELB_L:
        return "ELB_L";
    case BODY_PART::ELB_R:
        return "ELB_R";
    case BODY_PART::HIP_L:
        return "HIP_L";
    case BODY_PART::HIP_R:
        return "HIP_R";
    case BODY_PART::KNEE_L:
        return "KNEE_L";
    case BODY_PART::KNEE_R:
        return "KNEE_R";
    case BODY_PART::ANKLE_L:
        return "ANKLE_L";
    case BODY_PART::ANKLE_R:
        return "ANKLE_R";
    default:
        return "UNKNOWN";
    }
}
#include <sstream>

BodyControlSystem::BodyControlSystem(sf::RenderWindow *window)
{
    inputManager = std::make_unique<InputManager>(window);

    schemes.push_back(std::make_unique<SurgeonControlScheme>());
    schemes.push_back(std::make_unique<ChaosControlScheme>());
    schemes.push_back(std::make_unique<LearnControlScheme>());

    currentSchemeIndex = 2; // Start with Learn Mode (easiest)

    std::cout << "🎮 Body Control System Initialized!" << std::endl;
    std::cout << "Available Schemes:" << std::endl;
    for (size_t i = 0; i < schemes.size(); ++i)
    {
        std::cout << "  " << i << ": " << schemes[i]->getSchemeName() << std::endl;
    }
}

void BodyControlSystem::update(float dt)
{
    inputManager->update();

    if (inputManager->justPressed(BUTTON_ACTION::TOGGLE_SCHEME) && schemeToggleCooldown <= 0.0f)
    {
        nextScheme();
        schemeToggleCooldown = 0.5f; // Prevent rapid switching
    }

    if (schemeToggleCooldown > 0.0f)
    {
        schemeToggleCooldown -= dt;
    }

    // Toggle debug mode
    if (inputManager->justPressed(BUTTON_ACTION::PAUSE))
    {
        debugMode = !debugMode;
    }

    if (currentSchemeIndex < schemes.size())
    {
        schemes[currentSchemeIndex]->update(*inputManager, dt);
    }
}

void BodyControlSystem::applyControls(Jelly &body, uint64_t playerId, float dt)
{
    if (currentSchemeIndex < schemes.size())
    {
        schemes[currentSchemeIndex]->applyControls(body, playerId, dt);
    }
}

void BodyControlSystem::draw(sf::RenderWindow &window)
{
    if (!debugMode)
        return;

    sf::Vector2f debugPos = {10.0f, 10.0f};

    if (currentSchemeIndex < schemes.size())
    {
        auto *currentScheme = schemes[currentSchemeIndex].get();

        // For now, just draw debug info
        currentScheme->drawDebugInfo(window, {debugPos.x, debugPos.y + 40.0f});
    }
}

void BodyControlSystem::switchToScheme(int index)
{
    if (index >= 0 && index < schemes.size())
    {
        schemes[currentSchemeIndex]->reset(); // Reset old scheme
        currentSchemeIndex = index;
        std::cout << "🔄 Switched to: " << schemes[currentSchemeIndex]->getSchemeName() << std::endl;
    }
}

void BodyControlSystem::nextScheme()
{
    switchToScheme((currentSchemeIndex + 1) % schemes.size());
}

void BodyControlSystem::previousScheme()
{
    switchToScheme((currentSchemeIndex - 1 + schemes.size()) % schemes.size());
}

ControlSchemeBase *BodyControlSystem::getCurrentScheme() const
{
    if (currentSchemeIndex < schemes.size())
    {
        return schemes[currentSchemeIndex].get();
    }
    return nullptr;
}


void ControlSchemeBase::initializeBodyPart(BODY_PART part, float maxStamina, float drainRate, float recoveryRate)
{
    BodyPartControl control;
    control.part = part;
    control.forceDirection = {0.0f, 0.0f};
    control.intensity = 0.0f;
    control.isActive = false;
    control.stamina = maxStamina;
    control.maxStamina = maxStamina;
    control.staminaDrain = drainRate;
    control.staminaRecovery = recoveryRate;

    bodyControls[part] = control;
}

void ControlSchemeBase::updateStamina(BODY_PART part, bool isActive, float dt)
{
    auto it = bodyControls.find(part);
    if (it != bodyControls.end())
    {
        if (isActive && it->second.stamina > 0.0f)
        {
            it->second.stamina = std::max(0.0f, it->second.stamina - it->second.staminaDrain * dt);
        }
        else if (!isActive && it->second.stamina < it->second.maxStamina)
        {
            it->second.stamina = std::min(it->second.maxStamina, it->second.stamina + it->second.staminaRecovery * dt);
        }
    }
}

void ControlSchemeBase::applyForceToBodyPart(Jelly &body, uint64_t playerId, BODY_PART part, sf::Vector2f targetPos, float strength)
{
    // STABILITY-AWARE FORCE APPLICATION:

    auto it = bodyControls.find(part);
    if (it != bodyControls.end())
    {
        float staminaMultiplier = it->second.stamina / it->second.maxStamina;
        float finalStrength = strength * staminaMultiplier * globalStrengthMultiplier;

        // SMART PD TUNING: Assess stability and adjust gains accordingly
        float stabilityMetric = assessStabilityMetric(body, playerId);
        float stabilityAwareness = getStabilityAwareness(part);

        // Dynamic PD tuning based on stability state
        float baseKp = 40.0f;
        float baseKd = 12.0f;

        // If stability is high (> 0.8), allow more responsive forces
        if (stabilityMetric > 0.8f)
        {
            float responsivenessFactor = 1.0f + (stabilityMetric - 0.8f) * 2.0f; // Up to 1.4x at perfect stability
            baseKp = 40.0f * responsivenessFactor;
            baseKd = 12.0f * responsivenessFactor;
        }
        // If stability is low (< 0.5), reduce forces to maintain stability
        else if (stabilityMetric < 0.5f)
        {
            float cautionFactor = stabilityMetric / 0.5f; // 0 to 1 as stability approaches minimum
            baseKp = 40.0f * cautionFactor;
            baseKd = 12.0f * (cautionFactor + 0.5f); // More damping when unstable
        }

        float adaptiveKp = finalStrength * baseKp * stabilityAwareness;
        float adaptiveKd = finalStrength * baseKd * stabilityAwareness;

        body.set_part_target(playerId, part,
                             targetPos,  // Actual target position
                             adaptiveKp, // Stability-aware position gain
                             adaptiveKd  // Stability-aware damping
        );
    }
}

float ControlSchemeBase::getStabilityAwareness(BODY_PART part)
{
    // Return multiplier for stability-critical vs. mobility-focused parts

    switch (part)
    {
    // CORE STABILITY PARTS - Very conservative forces
    case BODY_PART::PELVIS:
    case BODY_PART::SPINE_LOW:
    case BODY_PART::SPINE_MID:
        return 0.3f; // Minimal interference with postural control

    case BODY_PART::FOOT_L:
    case BODY_PART::FOOT_R:
    case BODY_PART::ANKLE_L:
    case BODY_PART::ANKLE_R:
        return 0.4f; // Respect foot planting system

    // SUPPORT STRUCTURE - Moderate forces
    case BODY_PART::SPINE_UP:
    case BODY_PART::NECK:
    case BODY_PART::HIP_L:
    case BODY_PART::HIP_R:
        return 0.6f; // Some stability consideration

    case BODY_PART::KNEE_L:
    case BODY_PART::KNEE_R:
        return 0.7f; // Important for locomotion

    // EXPRESSIVE PARTS - Full forces allowed
    case BODY_PART::HEAD:
        return 0.8f; // Head movement for expression

    case BODY_PART::HAND_L:
    case BODY_PART::HAND_R:
    case BODY_PART::WRIST_L:
    case BODY_PART::WRIST_R:
    case BODY_PART::ELB_L:
    case BODY_PART::ELB_R:
    case BODY_PART::SHO_L:
    case BODY_PART::SHO_R:
        return 1.0f; // Full expressive control for arms/hands

    case BODY_PART::CLAV_L:
    case BODY_PART::CLAV_R:
        return 0.9f; // Near-full shoulder expression

    default:
        return 0.7f; // Conservative default
    }
}

float ControlSchemeBase::assessStabilityMetric(Jelly &body, uint64_t playerId)
{
    // SMART STABILITY ASSESSMENT: Evaluate multiple factors for overall stability

    float stabilityScore = 1.0f; // Start with perfect stability

    sf::Vector2f centerOfMass{0.f, 0.f};
    sf::Vector2f leftFoot{0.f, 0.f};
    sf::Vector2f rightFoot{0.f, 0.f};
    float totalMass = 0.f;
    bool hasLeftFoot = false, hasRightFoot = false;

    for (const auto &p : body.points)
    {
        if (p.id == playerId)
        {
            centerOfMass += p.pos * p.mass;
            totalMass += p.mass;

            if (p.body_part == BODY_PART::FOOT_L)
            {
                leftFoot = p.pos;
                hasLeftFoot = true;
            }
            else if (p.body_part == BODY_PART::FOOT_R)
            {
                rightFoot = p.pos;
                hasRightFoot = true;
            }
        }
    }

    if (totalMass > 0.f)
    {
        centerOfMass /= totalMass;

        // Factor 1: Balance over base of support
        if (hasLeftFoot && hasRightFoot)
        {
            sf::Vector2f baseCenter = (leftFoot + rightFoot) * 0.5f;
            float lateralOffset = std::abs(centerOfMass.x - baseCenter.x);
            float baseWidth = std::abs(leftFoot.x - rightFoot.x);

            if (baseWidth > 10.f) // Valid stance
            {
                float balanceRatio = lateralOffset / (baseWidth * 0.5f);
                balanceRatio = std::clamp(balanceRatio, 0.f, 2.f);
                stabilityScore *= (1.f - balanceRatio * 0.4f); // 40% penalty for imbalance
            }
        }

        float totalVelocity = 0.f;
        int pointCount = 0;

        for (const auto &p : body.points)
        {
            if (p.id == playerId && !p.locked)
            {
                sf::Vector2f vel = p.pos - p.prev_pos;
                totalVelocity += std::hypot(vel.x, vel.y);
                pointCount++;
            }
        }

        if (pointCount > 0)
        {
            float avgVelocity = totalVelocity / pointCount;
            float velocityFactor = std::clamp(1.f - (avgVelocity - 2.f) / 10.f, 0.2f, 1.f);
            stabilityScore *= velocityFactor; // Penalize excessive motion
        }

        // Factor 3: Spine alignment
        int spineUp = body.find_part_index(playerId, BODY_PART::SPINE_UP);
        int spineMid = body.find_part_index(playerId, BODY_PART::SPINE_MID);
        int spineLow = body.find_part_index(playerId, BODY_PART::SPINE_LOW);

        if (spineUp > 0 && spineMid > 0 && spineLow > 0)
        {
            sf::Vector2f upPos = body.points[spineUp].pos;
            sf::Vector2f midPos = body.points[spineMid].pos;
            sf::Vector2f lowPos = body.points[spineLow].pos;

            float spineHeight = upPos.y - lowPos.y;
            float midDeviation = std::abs(midPos.x - (upPos.x + lowPos.x) * 0.5f);

            if (spineHeight > 10.f)
            {
                float alignmentRatio = midDeviation / spineHeight;
                float alignmentFactor = std::clamp(1.f - alignmentRatio * 2.f, 0.5f, 1.f);
                stabilityScore *= alignmentFactor;
            }
        }
    }

    // Clamp final score between 0 and 1
    return std::clamp(stabilityScore, 0.f, 1.f);
}

void ControlSchemeBase::drawStaminaBar(sf::RenderWindow &window, sf::Vector2f position, float stamina, float maxStamina, sf::Color color)
{
    sf::RectangleShape background({60.0f, 8.0f});
    background.setPosition(position);
    background.setFillColor(sf::Color(50, 50, 50, 150));
    window.draw(background);

    float ratio = stamina / maxStamina;
    sf::RectangleShape bar({60.0f * ratio, 8.0f});
    bar.setPosition(position);

    // Color coding: green > yellow > red
    if (ratio > 0.6f)
    {
        bar.setFillColor(sf::Color::Green);
    }
    else if (ratio > 0.3f)
    {
        bar.setFillColor(sf::Color::Yellow);
    }
    else
    {
        bar.setFillColor(sf::Color::Red);
    }

    window.draw(bar);
}


SurgeonControlScheme::SurgeonControlScheme()
{
    initializeBodyPart(BODY_PART::SHO_L, 80.0f, 12.0f, 20.0f);   // Left shoulder
    initializeBodyPart(BODY_PART::ELB_L, 60.0f, 15.0f, 25.0f);   // Left elbow
    initializeBodyPart(BODY_PART::WRIST_L, 40.0f, 20.0f, 30.0f); // Left wrist
    initializeBodyPart(BODY_PART::SHO_R, 80.0f, 12.0f, 20.0f);   // Right shoulder
    initializeBodyPart(BODY_PART::ELB_R, 60.0f, 15.0f, 25.0f);   // Right elbow
    initializeBodyPart(BODY_PART::WRIST_R, 40.0f, 20.0f, 30.0f); // Right wrist

    initializeBodyPart(BODY_PART::HIP_L, 100.0f, 10.0f, 15.0f);  // Left hip
    initializeBodyPart(BODY_PART::KNEE_L, 80.0f, 12.0f, 18.0f);  // Left knee
    initializeBodyPart(BODY_PART::ANKLE_L, 60.0f, 15.0f, 22.0f); // Left ankle
    initializeBodyPart(BODY_PART::HIP_R, 100.0f, 10.0f, 15.0f);  // Right hip
    initializeBodyPart(BODY_PART::KNEE_R, 80.0f, 12.0f, 18.0f);  // Right knee
    initializeBodyPart(BODY_PART::ANKLE_R, 60.0f, 15.0f, 22.0f); // Right ankle

    initializeBodyPart(BODY_PART::PELVIS, 120.0f, 8.0f, 12.0f); // Core
    initializeBodyPart(BODY_PART::HEAD, 50.0f, 18.0f, 28.0f);   // Head

    // Define limb mappings with directional control
    limbMappings = {
        // Left arm
        {BUTTON_ACTION::ACTION_1, BUTTON_ACTION::ACTION_2, BODY_PART::SHO_L, {0.0f, -1.0f}, {0.0f, 1.0f}},
        {BUTTON_ACTION::ACTION_3, BUTTON_ACTION::ACTION_4, BODY_PART::ELB_L, {1.0f, 0.0f}, {-1.0f, 0.0f}},
        {BUTTON_ACTION::ACTION_5, BUTTON_ACTION::ACTION_6, BODY_PART::WRIST_L, {0.0f, -1.0f}, {0.0f, 1.0f}},

        // Right arm
        {BUTTON_ACTION::ACTION_7, BUTTON_ACTION::ACTION_8, BODY_PART::SHO_R, {0.0f, -1.0f}, {0.0f, 1.0f}},
        {BUTTON_ACTION::ACTION_9, BUTTON_ACTION::ACTION_10, BODY_PART::ELB_R, {-1.0f, 0.0f}, {1.0f, 0.0f}},
        {BUTTON_ACTION::ACTION_11, BUTTON_ACTION::ACTION_12, BODY_PART::WRIST_R, {0.0f, -1.0f}, {0.0f, 1.0f}},

        // Left leg
        {BUTTON_ACTION::ACTION_13, BUTTON_ACTION::ACTION_14, BODY_PART::HIP_L, {0.0f, -1.0f}, {0.0f, 1.0f}},
        {BUTTON_ACTION::ACTION_15, BUTTON_ACTION::ACTION_16, BODY_PART::KNEE_L, {1.0f, 0.0f}, {-1.0f, 0.0f}},
        {BUTTON_ACTION::ACTION_17, BUTTON_ACTION::ACTION_18, BODY_PART::ANKLE_L, {0.0f, -1.0f}, {0.0f, 1.0f}},

        // Right leg
        {BUTTON_ACTION::ACTION_19, BUTTON_ACTION::ACTION_20, BODY_PART::HIP_R, {0.0f, -1.0f}, {0.0f, 1.0f}},
        {BUTTON_ACTION::ACTION_21, BUTTON_ACTION::ACTION_22, BODY_PART::KNEE_R, {1.0f, 0.0f}, {-1.0f, 0.0f}},
        {BUTTON_ACTION::ACTION_23, BUTTON_ACTION::ACTION_24, BODY_PART::ANKLE_R, {0.0f, -1.0f}, {0.0f, 1.0f}}};
}

void SurgeonControlScheme::update(InputManager &input, float dt)
{
    for (const auto &mapping : limbMappings)
    {
        bool extendPressed = input.isHeld(mapping.extendAction);
        bool contractPressed = input.isHeld(mapping.contractAction);

        auto &control = bodyControls[mapping.bodyPart];

        if (extendPressed && control.stamina > 0.0f)
        {
            control.forceDirection = mapping.extendDirection;
            control.intensity = 1.0f;
            control.isActive = true;
        }
        else if (contractPressed && control.stamina > 0.0f)
        {
            control.forceDirection = mapping.contractDirection;
            control.intensity = 1.0f;
            control.isActive = true;
        }
        else
        {
            control.forceDirection = {0.0f, 0.0f};
            control.intensity = 0.0f;
            control.isActive = false;
        }

        updateStamina(mapping.bodyPart, control.isActive, dt);
    }

    // Balance assist
    balanceAssistEnabled = input.isHeld(BUTTON_ACTION::BALANCE_ASSIST);
}

void SurgeonControlScheme::applyControls(Jelly &body, uint64_t playerId, float dt)
{
    for (const auto &[part, control] : bodyControls)
    {
        if (control.isActive && control.intensity > 0.0f)
        {
            int partIndex = body.find_part_index(playerId, part);
            if (partIndex > 0)
            {
                sf::Vector2f currentPos = body.points[partIndex].pos;
                sf::Vector2f targetOffset = control.forceDirection * 50.0f * control.intensity;
                sf::Vector2f targetPos = currentPos + targetOffset;

                applyForceToBodyPart(body, playerId, part, targetPos, control.intensity);
            }
        }
    }
}

void SurgeonControlScheme::reset()
{
    for (auto &[part, control] : bodyControls)
    {
        control.stamina = control.maxStamina;
        control.isActive = false;
        control.intensity = 0.0f;
        control.forceDirection = {0.0f, 0.0f};
    }
}

std::string SurgeonControlScheme::getControlDescription() const
{
    return "Q/A: L.Shoulder | W/S: L.Elbow | E/D: L.Wrist\\n"
           "R/F: R.Shoulder | T/G: R.Elbow | Y/H: R.Wrist\\n"
           "U/J: L.Hip | I/K: L.Knee | O/L: L.Ankle\\n"
           "P/;: R.Hip | [/]: R.Knee | '/Enter: R.Ankle\\n"
           "Space: Jump | Shift: Balance Assist";
}

void SurgeonControlScheme::drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position)
{
    float yOffset = 0.0f;
    for (const auto &[part, control] : bodyControls)
    {
        drawStaminaBar(window, {position.x, position.y + yOffset},
                       control.stamina, control.maxStamina,
                       control.isActive ? sf::Color::Red : sf::Color::Blue);
        yOffset += 12.0f;
    }
}

bool SurgeonControlScheme::isControlActive(BODY_PART part) const
{
    auto it = bodyControls.find(part);
    return it != bodyControls.end() && it->second.isActive;
}

float SurgeonControlScheme::getControlIntensity(BODY_PART part) const
{
    auto it = bodyControls.find(part);
    return it != bodyControls.end() ? it->second.intensity : 0.0f;
}

float SurgeonControlScheme::getStamina(BODY_PART part) const
{
    auto it = bodyControls.find(part);
    return it != bodyControls.end() ? it->second.stamina : 0.0f;
}


ChaosControlScheme::ChaosControlScheme()
{
    initializeBodyPart(BODY_PART::SHO_L, 100.0f, 10.0f, 20.0f);
    initializeBodyPart(BODY_PART::SHO_R, 100.0f, 10.0f, 20.0f);
    initializeBodyPart(BODY_PART::HIP_L, 100.0f, 10.0f, 20.0f);
    initializeBodyPart(BODY_PART::HIP_R, 100.0f, 10.0f, 20.0f);
}

void ChaosControlScheme::update(InputManager &input, float dt)
{
    // Simplified implementation for now
    // TODO: Implement grouped control logic
}

void ChaosControlScheme::applyControls(Jelly &body, uint64_t playerId, float dt)
{
    // TODO: Implement grouped control application
}

void ChaosControlScheme::reset()
{
    for (auto &[part, control] : bodyControls)
    {
        control.stamina = control.maxStamina;
        control.isActive = false;
    }
}

std::string ChaosControlScheme::getControlDescription() const
{
    return "WASD: Left Arm | IJKL: Right Arm\\n"
           "TFGH: Left Leg | 7890: Right Leg\\n"
           "Space: Jump | Shift: Balance";
}

void ChaosControlScheme::drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position)
{
    // Basic debug info
}

bool ChaosControlScheme::isControlActive(BODY_PART part) const { return false; }
float ChaosControlScheme::getControlIntensity(BODY_PART part) const { return 0.0f; }
float ChaosControlScheme::getStamina(BODY_PART part) const { return 100.0f; }


LearnControlScheme::LearnControlScheme()
{
    selectableParts = {
        BODY_PART::PELVIS, BODY_PART::HEAD, // Start with stable core parts
        BODY_PART::SHO_L, BODY_PART::SHO_R, // Arms
        BODY_PART::ELB_L, BODY_PART::ELB_R,
        BODY_PART::HIP_L, BODY_PART::HIP_R,    // Hips (more stable than knees/ankles)
        BODY_PART::KNEE_L, BODY_PART::KNEE_R}; // Legs last (most destabilizing)

    for (auto part : selectableParts)
    {
        initializeBodyPart(part, 100.0f, 8.0f, 25.0f);
    }
}

void LearnControlScheme::update(InputManager &input, float dt)
{
    // Cycle through body parts
    if (input.justPressed(BUTTON_ACTION::TOGGLE_SCHEME) && partSwitchCooldown <= 0.0f)
    {
        currentPartIndex = (currentPartIndex + 1) % selectableParts.size();
        activePart = selectableParts[currentPartIndex];
        partSwitchCooldown = 0.3f;
    }

    if (partSwitchCooldown > 0.0f)
    {
        partSwitchCooldown -= dt;
    }

    // Control active part with WASD
    auto &control = bodyControls[activePart];
    bool anyInput = false;
    sf::Vector2f inputDirection = {0.0f, 0.0f};

    if (input.isHeld(BUTTON_ACTION::MOVE_LEFT))
    {
        inputDirection.x -= 1.0f;
        anyInput = true;
    }
    if (input.isHeld(BUTTON_ACTION::MOVE_RIGHT))
    {
        inputDirection.x += 1.0f;
        anyInput = true;
    }
    if (input.isHeld(BUTTON_ACTION::MOVE_UP))
    {
        inputDirection.y -= 1.0f;
        anyInput = true;
    }
    if (input.isHeld(BUTTON_ACTION::MOVE_DOWN))
    {
        inputDirection.y += 1.0f;
        anyInput = true;
    }

    if (anyInput && control.stamina > 0.0f)
    {
        float length = std::sqrt(inputDirection.x * inputDirection.x + inputDirection.y * inputDirection.y);
        if (length > 0.0f)
        {
            control.forceDirection = inputDirection / length;
            control.intensity = 1.0f;
            control.isActive = true;
        }
    }
    else
    {
        control.isActive = false;
        control.intensity = 0.0f;
    }

    updateStamina(activePart, control.isActive, dt);
}

void LearnControlScheme::applyControls(Jelly &body, uint64_t playerId, float dt)
{
    auto &control = bodyControls[activePart];
    if (control.isActive)
    {
        int partIndex = body.find_part_index(playerId, activePart);
        if (partIndex >= 0)
        {
            sf::Vector2f currentPos = body.points[partIndex].pos;

            sf::Vector2f targetOffset = control.forceDirection * 15.0f; // Much reduced for stability cooperation
            sf::Vector2f targetPos = currentPos + targetOffset;

            applyForceToBodyPart(body, playerId, activePart, targetPos, control.intensity * 0.2f); // Much lower intensity
        }
    }
    else
    {
        // CRITICAL: Clear part targets when not active to prevent interference with stability
        body.clear_part_target(playerId, activePart);
    }
}

void LearnControlScheme::reset()
{
    for (auto &[part, control] : bodyControls)
    {
        control.stamina = control.maxStamina;
        control.isActive = false;
    }
    currentPartIndex = 0;
    activePart = selectableParts[0];

    // CRITICAL: Clear any residual part targets that could interfere with stability
    // Note: We'll clear all targets for the player when Player spawns
}

std::string LearnControlScheme::getControlDescription() const
{
    return "Tab: Cycle Body Parts | WASD: Move Selected\\n"
           "Current: " +
           std::to_string(static_cast<int>(activePart)) +
           " (" + getBodyPartName(activePart) + ")" +
           " | Space: Jump | Shift: Balance";
}

void LearnControlScheme::drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position)
{
    // Highlight active part, show stamina
    auto &control = bodyControls[activePart];
    drawStaminaBar(window, position, control.stamina, control.maxStamina, sf::Color::Cyan);
}

bool LearnControlScheme::isControlActive(BODY_PART part) const
{
    return part == activePart && bodyControls.find(part) != bodyControls.end() &&
           bodyControls.at(part).isActive;
}

float LearnControlScheme::getControlIntensity(BODY_PART part) const
{
    if (part == activePart)
    {
        auto it = bodyControls.find(part);
        return it != bodyControls.end() ? it->second.intensity : 0.0f;
    }
    return 0.0f;
}

float LearnControlScheme::getStamina(BODY_PART part) const
{
    auto it = bodyControls.find(part);
    return it != bodyControls.end() ? it->second.stamina : 0.0f;
}
