#pragma once
#include "../libs.hpp"
#include "InputManager.hpp"
#include "../Gmae_physics/jelly.hpp"
#include <memory>

// Force vectors for body part control
struct BodyPartControl
{
    BODY_PART part;
    sf::Vector2f forceDirection;
    float intensity; // 0.0 to 1.0
    bool isActive;
    float stamina; // 0.0 to 100.0
    float maxStamina;
    float staminaDrain;    // per second when active
    float staminaRecovery; // per second when inactive
};

// Abstract base class for all control schemes
class ControlSchemeBase
{
public:
    virtual ~ControlSchemeBase() = default;

    // Core interface that all schemes must implement
    virtual void update(InputManager &input, float dt) = 0;
    virtual void applyControls(Jelly &body, uint64_t playerId, float dt) = 0;
    virtual void reset() = 0;

    // UI and feedback
    virtual std::string getSchemeName() const = 0;
    virtual std::string getControlDescription() const = 0;
    virtual void drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position) = 0;

    // Control state access
    virtual bool isControlActive(BODY_PART part) const = 0;
    virtual float getControlIntensity(BODY_PART part) const = 0;
    virtual float getStamina(BODY_PART part) const = 0;

protected:
    std::unordered_map<BODY_PART, BodyPartControl> bodyControls;
    bool balanceAssistEnabled = false;
    float globalStrengthMultiplier = 1.0f;

    // Helper functions for all schemes
    void initializeBodyPart(BODY_PART part, float maxStamina = 100.0f, float drainRate = 15.0f, float recoveryRate = 25.0f);
    void updateStamina(BODY_PART part, bool isActive, float dt);
    void applyForceToBodyPart(Jelly &body, uint64_t playerId, BODY_PART part, sf::Vector2f force, float strength);
    void drawStaminaBar(sf::RenderWindow &window, sf::Vector2f position, float stamina, float maxStamina, sf::Color color);

    // STABILITY-AWARE CONTROL: Get force multiplier based on part's stability importance
    float getStabilityAwareness(BODY_PART part);

    // SMART PD TUNING: Assess current stability to adapt force gains
    float assessStabilityMetric(Jelly &body, uint64_t playerId);
};

// Advanced "Surgeon Mode" - Maximum control and complexity
class SurgeonControlScheme : public ControlSchemeBase
{
private:
    struct LimbPair
    {
        BUTTON_ACTION extendAction;
        BUTTON_ACTION contractAction;
        BODY_PART bodyPart;
        sf::Vector2f extendDirection;
        sf::Vector2f contractDirection;
    };

    std::vector<LimbPair> limbMappings;

public:
    SurgeonControlScheme();

    void update(InputManager &input, float dt) override;
    void applyControls(Jelly &body, uint64_t playerId, float dt) override;
    void reset() override;

    std::string getSchemeName() const override { return "Surgeon Mode"; }
    std::string getControlDescription() const override;
    void drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position) override;

    bool isControlActive(BODY_PART part) const override;
    float getControlIntensity(BODY_PART part) const override;
    float getStamina(BODY_PART part) const override;
};

// Simplified "Chaos Mode" - Grouped controls, still challenging
class ChaosControlScheme : public ControlSchemeBase
{
private:
    struct LimbGroup
    {
        std::vector<BUTTON_ACTION> actions;
        std::vector<BODY_PART> bodyParts;
        std::string groupName;
    };

    std::vector<LimbGroup> limbGroups;

public:
    ChaosControlScheme();

    void update(InputManager &input, float dt) override;
    void applyControls(Jelly &body, uint64_t playerId, float dt) override;
    void reset() override;

    std::string getSchemeName() const override { return "Chaos Mode"; }
    std::string getControlDescription() const override;
    void drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position) override;

    bool isControlActive(BODY_PART part) const override;
    float getControlIntensity(BODY_PART part) const override;
    float getStamina(BODY_PART part) const override;
};

// Contextual "Learn Mode" - Tab through body parts
class LearnControlScheme : public ControlSchemeBase
{
private:
    std::vector<BODY_PART> selectableParts;
    int currentPartIndex = 0;
    BODY_PART activePart = BODY_PART::PELVIS;
    float partSwitchCooldown = 0.0f;
    bool showingHelp = false;

public:
    LearnControlScheme();

    void update(InputManager &input, float dt) override;
    void applyControls(Jelly &body, uint64_t playerId, float dt) override;
    void reset() override;

    std::string getSchemeName() const override { return "Learn Mode"; }
    std::string getControlDescription() const override;
    void drawDebugInfo(sf::RenderWindow &window, sf::Vector2f position) override;

    bool isControlActive(BODY_PART part) const override;
    float getControlIntensity(BODY_PART part) const override;
    float getStamina(BODY_PART part) const override;

    BODY_PART getActivePart() const { return activePart; }
};

// Master control system that manages all schemes
class BodyControlSystem
{
private:
    std::unique_ptr<InputManager> inputManager;
    std::vector<std::unique_ptr<ControlSchemeBase>> schemes;
    int currentSchemeIndex = 0;
    float schemeToggleCooldown = 0.0f;

    // Global settings
    bool debugMode = false;
    bool tutorialMode = false;
    float masterVolume = 1.0f;

public:
    BodyControlSystem(sf::RenderWindow *window);

    void update(float dt);
    void applyControls(Jelly &body, uint64_t playerId, float dt);
    void draw(sf::RenderWindow &window);

    // Scheme management
    void switchToScheme(int index);
    void nextScheme();
    void previousScheme();
    ControlSchemeBase *getCurrentScheme() const;

    // Settings
    void setDebugMode(bool enabled) { debugMode = enabled; }
    void setTutorialMode(bool enabled) { tutorialMode = enabled; }
    bool isDebugMode() const { return debugMode; }

    // Input access
    InputManager &getInputManager() { return *inputManager; }
};
