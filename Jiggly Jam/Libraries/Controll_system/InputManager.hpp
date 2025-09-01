#pragma once
#include "../libs.hpp"
#include <unordered_map>
#include <functional>

// Universal Input System - supports keyboard, gamepad, custom mappings
enum class INPUT_DEVICE
{
    KEYBOARD,
    GAMEPAD_1,
    GAMEPAD_2,
    CUSTOM
};

enum class BUTTON_ACTION
{
    // Movement controls
    MOVE_UP,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,

    // Body part controls (generic - mapped by control schemes)
    ACTION_1,
    ACTION_2,
    ACTION_3,
    ACTION_4,
    ACTION_5,
    ACTION_6,
    ACTION_7,
    ACTION_8,
    ACTION_9,
    ACTION_10,
    ACTION_11,
    ACTION_12,
    ACTION_13,
    ACTION_14,
    ACTION_15,
    ACTION_16,
    ACTION_17,
    ACTION_18,
    ACTION_19,
    ACTION_20,
    ACTION_21,
    ACTION_22,
    ACTION_23,
    ACTION_24,

    // System controls
    JUMP,
    BALANCE_ASSIST,
    TOGGLE_SCHEME,
    PAUSE,
    RESET,

    // Modifiers
    MODIFIER_1,
    MODIFIER_2,
    MODIFIER_3
};

enum class BUTTON_STATE
{
    RELEASED,
    PRESSED,
    HELD,
    JUST_RELEASED
};

struct ButtonMapping
{
    sf::Keyboard::Key keyboardKey = sf::Keyboard::Unknown;
    int gamepadButton = -1;
    int gamepadAxis = -1;
    float axisThreshold = 0.5f;
    bool isAnalog = false;
};

class InputManager
{
private:
    std::unordered_map<BUTTON_ACTION, ButtonMapping> buttonMappings;
    std::unordered_map<BUTTON_ACTION, BUTTON_STATE> buttonStates;
    std::unordered_map<BUTTON_ACTION, float> analogValues;

    INPUT_DEVICE activeDevice = INPUT_DEVICE::KEYBOARD;
    sf::RenderWindow *window = nullptr;

    // Previous frame states for detecting transitions
    std::unordered_map<BUTTON_ACTION, bool> prevPressed;

public:
    InputManager(sf::RenderWindow *win) : window(win)
    {
        initializeDefaultMappings();
    }

    // Initialize default control mappings
    void initializeDefaultMappings();

    // Update input states each frame
    void update();

    // Query button states
    bool isPressed(BUTTON_ACTION action) const;
    bool isHeld(BUTTON_ACTION action) const;
    bool justPressed(BUTTON_ACTION action) const;
    bool justReleased(BUTTON_ACTION action) const;
    float getAnalogValue(BUTTON_ACTION action) const;

    // Dynamic remapping
    void remapButton(BUTTON_ACTION action, sf::Keyboard::Key key);
    void remapButton(BUTTON_ACTION action, int gamepadButton);
    void remapAnalogStick(BUTTON_ACTION action, int gamepadAxis, float threshold = 0.5f);

    // Device management
    void setActiveDevice(INPUT_DEVICE device) { activeDevice = device; }
    INPUT_DEVICE getActiveDevice() const { return activeDevice; }

    // Save/load control schemes
    void saveScheme(const std::string &filename);
    void loadScheme(const std::string &filename);
};
