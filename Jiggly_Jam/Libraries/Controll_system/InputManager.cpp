#include "InputManager.hpp"
#include <fstream>
#include <iostream>

void InputManager::initializeDefaultMappings()
{
    buttonMappings[BUTTON_ACTION::MOVE_UP] = {sf::Keyboard::W, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::MOVE_DOWN] = {sf::Keyboard::S, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::MOVE_LEFT] = {sf::Keyboard::A, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::MOVE_RIGHT] = {sf::Keyboard::D, -1, -1, 0.5f, false};

    buttonMappings[BUTTON_ACTION::ACTION_1] = {sf::Keyboard::Q, -1, -1, 0.5f, false}; // Left shoulder up
    buttonMappings[BUTTON_ACTION::ACTION_2] = {sf::Keyboard::A, -1, -1, 0.5f, false}; // Left shoulder down
    buttonMappings[BUTTON_ACTION::ACTION_3] = {sf::Keyboard::W, -1, -1, 0.5f, false}; // Left elbow extend
    buttonMappings[BUTTON_ACTION::ACTION_4] = {sf::Keyboard::S, -1, -1, 0.5f, false}; // Left elbow contract
    buttonMappings[BUTTON_ACTION::ACTION_5] = {sf::Keyboard::E, -1, -1, 0.5f, false}; // Left wrist up
    buttonMappings[BUTTON_ACTION::ACTION_6] = {sf::Keyboard::D, -1, -1, 0.5f, false}; // Left wrist down

    buttonMappings[BUTTON_ACTION::ACTION_7] = {sf::Keyboard::R, -1, -1, 0.5f, false};  // Right shoulder up
    buttonMappings[BUTTON_ACTION::ACTION_8] = {sf::Keyboard::F, -1, -1, 0.5f, false};  // Right shoulder down
    buttonMappings[BUTTON_ACTION::ACTION_9] = {sf::Keyboard::T, -1, -1, 0.5f, false};  // Right elbow extend
    buttonMappings[BUTTON_ACTION::ACTION_10] = {sf::Keyboard::G, -1, -1, 0.5f, false}; // Right elbow contract
    buttonMappings[BUTTON_ACTION::ACTION_11] = {sf::Keyboard::Y, -1, -1, 0.5f, false}; // Right wrist up
    buttonMappings[BUTTON_ACTION::ACTION_12] = {sf::Keyboard::H, -1, -1, 0.5f, false}; // Right wrist down

    buttonMappings[BUTTON_ACTION::ACTION_13] = {sf::Keyboard::U, -1, -1, 0.5f, false}; // Left hip forward
    buttonMappings[BUTTON_ACTION::ACTION_14] = {sf::Keyboard::J, -1, -1, 0.5f, false}; // Left hip back
    buttonMappings[BUTTON_ACTION::ACTION_15] = {sf::Keyboard::I, -1, -1, 0.5f, false}; // Left knee extend
    buttonMappings[BUTTON_ACTION::ACTION_16] = {sf::Keyboard::K, -1, -1, 0.5f, false}; // Left knee contract
    buttonMappings[BUTTON_ACTION::ACTION_17] = {sf::Keyboard::O, -1, -1, 0.5f, false}; // Left ankle up
    buttonMappings[BUTTON_ACTION::ACTION_18] = {sf::Keyboard::L, -1, -1, 0.5f, false}; // Left ankle down

    buttonMappings[BUTTON_ACTION::ACTION_19] = {sf::Keyboard::P, -1, -1, 0.5f, false};         // Right hip forward
    buttonMappings[BUTTON_ACTION::ACTION_20] = {sf::Keyboard::Semicolon, -1, -1, 0.5f, false}; // Right hip back
    buttonMappings[BUTTON_ACTION::ACTION_21] = {sf::Keyboard::LBracket, -1, -1, 0.5f, false};  // Right knee extend
    buttonMappings[BUTTON_ACTION::ACTION_22] = {sf::Keyboard::Quote, -1, -1, 0.5f, false};     // Right knee contract
    buttonMappings[BUTTON_ACTION::ACTION_23] = {sf::Keyboard::RBracket, -1, -1, 0.5f, false};  // Right ankle up
    buttonMappings[BUTTON_ACTION::ACTION_24] = {sf::Keyboard::Enter, -1, -1, 0.5f, false};     // Right ankle down

    // System controls
    buttonMappings[BUTTON_ACTION::JUMP] = {sf::Keyboard::Space, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::BALANCE_ASSIST] = {sf::Keyboard::LShift, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::TOGGLE_SCHEME] = {sf::Keyboard::Tab, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::PAUSE] = {sf::Keyboard::Escape, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::RESET] = {sf::Keyboard::Backspace, -1, -1, 0.5f, false};

    // Modifiers
    buttonMappings[BUTTON_ACTION::MODIFIER_1] = {sf::Keyboard::LControl, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::MODIFIER_2] = {sf::Keyboard::LAlt, -1, -1, 0.5f, false};
    buttonMappings[BUTTON_ACTION::MODIFIER_3] = {sf::Keyboard::RShift, -1, -1, 0.5f, false};
}

void InputManager::update()
{
    // Store previous states
    for (auto &[action, state] : buttonStates)
    {
        prevPressed[action] = (state == BUTTON_STATE::PRESSED || state == BUTTON_STATE::HELD);
    }

    for (auto &[action, mapping] : buttonMappings)
    {
        bool currentlyPressed = false;
        float analogValue = 0.0f;

        if (activeDevice == INPUT_DEVICE::KEYBOARD)
        {
            if (mapping.keyboardKey != sf::Keyboard::Unknown)
            {
                currentlyPressed = sf::Keyboard::isKeyPressed(mapping.keyboardKey);
            }
        }
        // TODO: Add gamepad support here

        // Determine button state based on current and previous
        bool wasPressed = prevPressed[action];

        if (currentlyPressed && !wasPressed)
        {
            buttonStates[action] = BUTTON_STATE::PRESSED;
        }
        else if (currentlyPressed && wasPressed)
        {
            buttonStates[action] = BUTTON_STATE::HELD;
        }
        else if (!currentlyPressed && wasPressed)
        {
            buttonStates[action] = BUTTON_STATE::JUST_RELEASED;
        }
        else
        {
            buttonStates[action] = BUTTON_STATE::RELEASED;
        }

        analogValues[action] = analogValue;
    }
}

bool InputManager::isPressed(BUTTON_ACTION action) const
{
    auto it = buttonStates.find(action);
    return it != buttonStates.end() && it->second == BUTTON_STATE::PRESSED;
}

bool InputManager::isHeld(BUTTON_ACTION action) const
{
    auto it = buttonStates.find(action);
    return it != buttonStates.end() && (it->second == BUTTON_STATE::PRESSED || it->second == BUTTON_STATE::HELD);
}

bool InputManager::justPressed(BUTTON_ACTION action) const
{
    auto it = buttonStates.find(action);
    return it != buttonStates.end() && it->second == BUTTON_STATE::PRESSED;
}

bool InputManager::justReleased(BUTTON_ACTION action) const
{
    auto it = buttonStates.find(action);
    return it != buttonStates.end() && it->second == BUTTON_STATE::JUST_RELEASED;
}

float InputManager::getAnalogValue(BUTTON_ACTION action) const
{
    auto it = analogValues.find(action);
    return it != analogValues.end() ? it->second : 0.0f;
}

void InputManager::remapButton(BUTTON_ACTION action, sf::Keyboard::Key key)
{
    buttonMappings[action].keyboardKey = key;
}

void InputManager::remapButton(BUTTON_ACTION action, int gamepadButton)
{
    buttonMappings[action].gamepadButton = gamepadButton;
}

void InputManager::remapAnalogStick(BUTTON_ACTION action, int gamepadAxis, float threshold)
{
    buttonMappings[action].gamepadAxis = gamepadAxis;
    buttonMappings[action].axisThreshold = threshold;
    buttonMappings[action].isAnalog = true;
}

void InputManager::saveScheme(const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        for (const auto &[action, mapping] : buttonMappings)
        {
            file << static_cast<int>(action) << " "
                 << static_cast<int>(mapping.keyboardKey) << " "
                 << mapping.gamepadButton << " "
                 << mapping.gamepadAxis << " "
                 << mapping.axisThreshold << " "
                 << mapping.isAnalog << std::endl;
        }
        file.close();
        std::cout << "Control scheme saved to " << filename << std::endl;
    }
}

void InputManager::loadScheme(const std::string &filename)
{
    std::ifstream file(filename);
    if (file.is_open())
    {
        int actionInt, keyInt, gamepadButton, gamepadAxis;
        float axisThreshold;
        bool isAnalog;

        while (file >> actionInt >> keyInt >> gamepadButton >> gamepadAxis >> axisThreshold >> isAnalog)
        {
            BUTTON_ACTION action = static_cast<BUTTON_ACTION>(actionInt);
            ButtonMapping mapping;
            mapping.keyboardKey = static_cast<sf::Keyboard::Key>(keyInt);
            mapping.gamepadButton = gamepadButton;
            mapping.gamepadAxis = gamepadAxis;
            mapping.axisThreshold = axisThreshold;
            mapping.isAnalog = isAnalog;

            buttonMappings[action] = mapping;
        }
        file.close();
        std::cout << "Control scheme loaded from " << filename << std::endl;
    }
}
