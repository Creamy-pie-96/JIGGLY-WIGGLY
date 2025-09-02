#include "settings_perser.hpp"
#include <algorithm>
#include <sstream>

SettingsParser::SettingsParser()
{
    applyDefaultSettings();
}

SettingsParser::~SettingsParser()
{
    // No cleanup needed
}

bool SettingsParser::loadSettings(const std::string &filepath)
{
    filepath_cache = filepath;

    std::ifstream file(filepath);
    if (!file.is_open())
    {
        last_error = "Could not open settings file: " + filepath;
        std::cout << "⚠️ " << last_error << " - Using default settings." << std::endl;
        applyDefaultSettings();
        return false;
    }

    std::cout << "🎮 Loading Gang Beasts physics settings from: " << filepath << std::endl;

    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '/' || line[0] == '#')
        {
            continue;
        }

        // Look for main sections
        if (line.find("\"physics_personality\"") != std::string::npos)
        {
            parsePhysicsPersonality(file);
        }
        else if (line.find("\"postural_stability\"") != std::string::npos)
        {
            parsePosturalStability(file);
        }
        else if (line.find("\"advanced_spring_system\"") != std::string::npos)
        {
            parseAdvancedSpringSystem(file);
        }
        /*
        else if (line.find("\"physics_driven_walking\"") != std::string::npos)
        {
            parsePhysicsDrivenWalking(file);
        }
        */
        else if (line.find("\"performance\"") != std::string::npos)
        {
            parsePerformanceSettings(file);
        }
        else if (line.find("\"debug\"") != std::string::npos)
        {
            parseDebugSettings(file);
        }
    }

    file.close();
    settings.settings_loaded = true;

    std::cout << "✅ Gang Beasts physics settings loaded successfully!" << std::endl;
    std::cout << "🎯 Physics Personality: " << (settings.physics_personality.skeleton_springs.base_stiffness_multiplier * 100) << "% skeleton stiffness" << std::endl;
    std::cout << "⚖️ Postural Stability: " << (settings.postural_stability.reaction_timing.base_reaction_delay * 1000) << "ms reaction delay" << std::endl;

    return true;
}

void SettingsParser::parsePhysicsPersonality(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break; // End of physics_personality section
        }

        if (line.find("\"skeleton_springs\"") != std::string::npos)
        {
            parseSkeletonSprings(file);
        }
        else if (line.find("\"flesh_springs\"") != std::string::npos)
        {
            parseFleshSprings(file);
        }
        else if (line.find("\"mass_distribution\"") != std::string::npos)
        {
            parseMassDistribution(file);
        }
        else if (line.find("\"spring_fatigue\"") != std::string::npos)
        {
            parseSpringFatigue(file);
        }
    }
}

void SettingsParser::parseSkeletonSprings(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "base_stiffness_multiplier")
            {
                settings.physics_personality.skeleton_springs.base_stiffness_multiplier = parseFloat(value, 0.65f);
            }
            else if (key == "spine_joints")
            {
                settings.physics_personality.skeleton_springs.spine_joints = parseFloat(value, 0.7f);
            }
            else if (key == "arm_joints")
            {
                settings.physics_personality.skeleton_springs.arm_joints = parseFloat(value, 0.5f);
            }
            else if (key == "leg_joints")
            {
                settings.physics_personality.skeleton_springs.leg_joints = parseFloat(value, 0.8f);
            }
            else if (key == "neck_joint")
            {
                settings.physics_personality.skeleton_springs.neck_joint = parseFloat(value, 0.6f);
            }
            else if (key == "wrist_joints")
            {
                settings.physics_personality.skeleton_springs.wrist_joints = parseFloat(value, 0.4f);
            }
            else if (key == "ankle_joints")
            {
                settings.physics_personality.skeleton_springs.ankle_joints = parseFloat(value, 0.75f);
            }
        }
    }
}

void SettingsParser::parseFleshSprings(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "damping_multiplier")
            {
                settings.physics_personality.flesh_springs.damping_multiplier = parseFloat(value, 1.4f);
            }
            else if (key == "stiffness_reduction")
            {
                settings.physics_personality.flesh_springs.stiffness_reduction = parseFloat(value, 0.25f);
            }
            else if (key == "wobble_frequency")
            {
                settings.physics_personality.flesh_springs.wobble_frequency = parseFloat(value, 0.85f);
            }
            else if (key == "wobble_amplitude")
            {
                settings.physics_personality.flesh_springs.wobble_amplitude = parseFloat(value, 1.2f);
            }
        }
    }
}

void SettingsParser::parseMassDistribution(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "head_mass_multiplier")
            {
                settings.physics_personality.mass_distribution.head_mass_multiplier = parseFloat(value, 1.3f);
            }
            else if (key == "torso_mass_multiplier")
            {
                settings.physics_personality.mass_distribution.torso_mass_multiplier = parseFloat(value, 1.1f);
            }
            else if (key == "limb_mass_multiplier")
            {
                settings.physics_personality.mass_distribution.limb_mass_multiplier = parseFloat(value, 0.9f);
            }
        }
    }
}

void SettingsParser::parseSpringFatigue(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.physics_personality.spring_fatigue.enabled = parseBool(value, true);
            }
            else if (key == "fatigue_rate")
            {
                settings.physics_personality.spring_fatigue.fatigue_rate = parseFloat(value, 0.02f);
            }
            else if (key == "recovery_rate")
            {
                settings.physics_personality.spring_fatigue.recovery_rate = parseFloat(value, 0.01f);
            }
            else if (key == "min_strength")
            {
                settings.physics_personality.spring_fatigue.min_strength = parseFloat(value, 0.6f);
            }
            else if (key == "max_strength")
            {
                settings.physics_personality.spring_fatigue.max_strength = parseFloat(value, 1.0f);
            }
        }
    }
}

void SettingsParser::parsePosturalStability(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        if (line.find("\"reaction_timing\"") != std::string::npos)
        {
            parseReactionTiming(file);
        }
        else if (line.find("\"overcompensation\"") != std::string::npos)
        {
            parseOvercompensation(file);
        }
        else if (line.find("\"sensitivity\"") != std::string::npos)
        {
            parseSensitivity(file);
        }
        else if (line.find("\"comedy_modes\"") != std::string::npos)
        {
            parseComedyModes(file);
        }
    }
}

void SettingsParser::parseReactionTiming(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "base_reaction_delay")
            {
                settings.postural_stability.reaction_timing.base_reaction_delay = parseFloat(value, 0.15f);
            }
            else if (key == "panic_reaction_delay")
            {
                settings.postural_stability.reaction_timing.panic_reaction_delay = parseFloat(value, 0.08f);
            }
            else if (key == "recovery_acceleration")
            {
                settings.postural_stability.reaction_timing.recovery_acceleration = parseFloat(value, 2.5f);
            }
        }
    }
}

void SettingsParser::parseOvercompensation(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.postural_stability.overcompensation.enabled = parseBool(value, true);
            }
            else if (key == "overcompensation_factor")
            {
                settings.postural_stability.overcompensation.overcompensation_factor = parseFloat(value, 1.8f);
            }
            else if (key == "oscillation_damping")
            {
                settings.postural_stability.overcompensation.oscillation_damping = parseFloat(value, 0.7f);
            }
            else if (key == "wobble_decay_time")
            {
                settings.postural_stability.overcompensation.wobble_decay_time = parseFloat(value, 2.0f);
            }
        }
    }
}

void SettingsParser::parseSensitivity(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "balance_threshold")
            {
                settings.postural_stability.sensitivity.balance_threshold = parseFloat(value, 0.12f);
            }
            else if (key == "panic_threshold")
            {
                settings.postural_stability.sensitivity.panic_threshold = parseFloat(value, 0.25f);
            }
            else if (key == "recovery_threshold")
            {
                settings.postural_stability.sensitivity.recovery_threshold = parseFloat(value, 0.05f);
            }
            else if (key == "ground_detection_tolerance")
            {
                settings.postural_stability.sensitivity.ground_detection_tolerance = parseFloat(value, 8.0f);
            }
        }
    }
}

void SettingsParser::parseComedyModes(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        // Parse drunk_walking, nervous_character, confident_character
        // For now, just parse the enabled flags - full implementation can be expanded
        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled" && line.find("drunk_walking") != std::string::npos)
            {
                settings.postural_stability.comedy_modes.drunk_walking.enabled = parseBool(value, false);
            }
        }
    }
}

void SettingsParser::parsePerformanceSettings(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "physics_substeps")
            {
                settings.performance.physics_substeps = parseInt(value, 3);
            }
            else if (key == "stability_update_frequency")
            {
                settings.performance.stability_update_frequency = parseInt(value, 60);
            }
            else if (key == "spring_animation_precision")
            {
                settings.performance.spring_animation_precision = parseFloat(value, 0.01f);
            }
            else if (key == "force_calculation_threshold")
            {
                settings.performance.force_calculation_threshold = parseFloat(value, 0.1f);
            }
        }
    }
}

void SettingsParser::parseDebugSettings(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "show_stability_forces")
            {
                settings.debug.show_stability_forces = parseBool(value, false);
            }
            else if (key == "show_spring_stress")
            {
                settings.debug.show_spring_stress = parseBool(value, false);
            }
            else if (key == "log_physics_events")
            {
                settings.debug.log_physics_events = parseBool(value, false);
            }
        }
    }
}

// Step 1.3: Advanced Spring System Parser
void SettingsParser::parseAdvancedSpringSystem(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        if (line.find("\"context_aware_stiffness\"") != std::string::npos)
        {
            parseContextAwareStiffness(file);
        }
        else if (line.find("\"spring_animation_system\"") != std::string::npos)
        {
            parseSpringAnimationSystem(file);
        }
        else if (line.find("\"spring_strength_modulation\"") != std::string::npos)
        {
            parseSpringStrengthModulation(file);
        }
        else if (line.find("\"adaptive_physics\"") != std::string::npos)
        {
            parseAdaptivePhysics(file);
        }
    }
}

void SettingsParser::parseContextAwareStiffness(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.advanced_spring_system.context_aware_stiffness.enabled = parseBool(value, true);
            }
            else if (key == "walking_stiffness_multiplier")
            {
                settings.advanced_spring_system.context_aware_stiffness.walking_stiffness_multiplier = parseFloat(value, 1.0f);
            }
            else if (key == "jumping_stiffness_multiplier")
            {
                settings.advanced_spring_system.context_aware_stiffness.jumping_stiffness_multiplier = parseFloat(value, 1.0f);
            }
            else if (key == "falling_stiffness_multiplier")
            {
                settings.advanced_spring_system.context_aware_stiffness.falling_stiffness_multiplier = parseFloat(value, 1.0f);
            }
            else if (key == "grabbing_stiffness_multiplier")
            {
                settings.advanced_spring_system.context_aware_stiffness.grabbing_stiffness_multiplier = parseFloat(value, 1.0f);
            }
            else if (key == "idle_stiffness_multiplier")
            {
                settings.advanced_spring_system.context_aware_stiffness.idle_stiffness_multiplier = parseFloat(value, 1.0f);
            }
            else if (key == "transition_speed")
            {
                settings.advanced_spring_system.context_aware_stiffness.transition_speed = parseFloat(value, 2.0f);
            }
        }
    }
}

void SettingsParser::parseSpringAnimationSystem(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.advanced_spring_system.spring_animation_system.enabled = parseBool(value, true);
            }
            else if (key == "max_concurrent_animations")
            {
                settings.advanced_spring_system.spring_animation_system.max_concurrent_animations = parseInt(value, 8);
            }
            else if (key == "animation_blend_time")
            {
                settings.advanced_spring_system.spring_animation_system.animation_blend_time = parseFloat(value, 0.3f);
            }
            else if (key == "keyframe_interpolation")
            {
                // For string values, remove quotes
                if (value.front() == '"' && value.back() == '"')
                {
                    value = value.substr(1, value.length() - 2);
                }
                settings.advanced_spring_system.spring_animation_system.keyframe_interpolation = value;
            }
            // Priority system parsing (simplified for now)
            else if (key == "walking_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.walking_priority = parseInt(value, 3);
            }
            else if (key == "jumping_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.jumping_priority = parseInt(value, 5);
            }
            else if (key == "falling_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.falling_priority = parseInt(value, 7);
            }
            else if (key == "grabbing_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.grabbing_priority = parseInt(value, 6);
            }
            else if (key == "waving_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.waving_priority = parseInt(value, 2);
            }
            else if (key == "idle_priority")
            {
                settings.advanced_spring_system.spring_animation_system.priority_system.idle_priority = parseInt(value, 1);
            }
        }
    }
}

void SettingsParser::parseSpringStrengthModulation(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.advanced_spring_system.spring_strength_modulation.enabled = parseBool(value, true);
            }
            else if (key == "modulation_smoothing")
            {
                settings.advanced_spring_system.spring_strength_modulation.modulation_smoothing = parseFloat(value, 3.0f);
            }
            // Action strength factors
            else if (key == "walking_leg_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.walking_leg_strength = parseFloat(value, 1.0f);
            }
            else if (key == "walking_arm_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.walking_arm_strength = parseFloat(value, 1.0f);
            }
            else if (key == "jumping_leg_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.jumping_leg_strength = parseFloat(value, 1.0f);
            }
            else if (key == "landing_leg_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.landing_leg_strength = parseFloat(value, 1.0f);
            }
            else if (key == "grabbing_arm_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.grabbing_arm_strength = parseFloat(value, 1.0f);
            }
            else if (key == "waving_arm_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.waving_arm_strength = parseFloat(value, 1.0f);
            }
            else if (key == "balancing_core_strength")
            {
                settings.advanced_spring_system.spring_strength_modulation.action_strength_factors.balancing_core_strength = parseFloat(value, 1.0f);
            }
        }
    }
}

void SettingsParser::parseAdaptivePhysics(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.advanced_spring_system.adaptive_physics.enabled = parseBool(value, true);
            }
            else if (key == "learning_rate")
            {
                settings.advanced_spring_system.adaptive_physics.learning_rate = parseFloat(value, 0.0f);
            }
            else if (key == "adaptation_memory")
            {
                settings.advanced_spring_system.adaptive_physics.adaptation_memory = parseFloat(value, 5.0f);
            }
            // Stress response
            else if (key == "stress_enabled" || (key == "enabled" && line.find("stress_response") != std::string::npos))
            {
                settings.advanced_spring_system.adaptive_physics.stress_response.enabled = parseBool(value, false);
            }
            else if (key == "stress_threshold")
            {
                settings.advanced_spring_system.adaptive_physics.stress_response.stress_threshold = parseFloat(value, 2.0f);
            }
            else if (key == "recovery_time")
            {
                settings.advanced_spring_system.adaptive_physics.stress_response.recovery_time = parseFloat(value, 3.0f);
            }
            else if (key == "stress_weakening")
            {
                settings.advanced_spring_system.adaptive_physics.stress_response.stress_weakening = parseFloat(value, 0.0f);
            }
        }
    }
}

// Helper functions
std::string SettingsParser::trim(const std::string &str)
{
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
        return "";
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

bool SettingsParser::parseLine(const std::string &line, std::string &key, std::string &value)
{
    size_t colonPos = line.find(':');
    if (colonPos == std::string::npos)
        return false;

    key = trim(line.substr(0, colonPos));
    value = trim(line.substr(colonPos + 1));

    // Remove quotes from key and value
    if (key.front() == '"' && key.back() == '"')
    {
        key = key.substr(1, key.length() - 2);
    }

    // Remove trailing comma and quotes from value
    if (!value.empty() && value.back() == ',')
    {
        value = value.substr(0, value.length() - 1);
    }
    value = trim(value);

    return true;
}

float SettingsParser::parseFloat(const std::string &value, float defaultValue)
{
    try
    {
        return std::stof(value);
    }
    catch (...)
    {
        return defaultValue;
    }
}

bool SettingsParser::parseBool(const std::string &value, bool defaultValue)
{
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "true")
        return true;
    if (lower == "false")
        return false;

    return defaultValue;
}

int SettingsParser::parseInt(const std::string &value, int defaultValue)
{
    try
    {
        return std::stoi(value);
    }
    catch (...)
    {
        return defaultValue;
    }
}

const GangBeastsSettings::GangBeastsPhysicsSettings &SettingsParser::getSettings() const
{
    return settings;
}

bool SettingsParser::isLoaded() const
{
    return settings.settings_loaded;
}

const std::string &SettingsParser::getLastError() const
{
    return last_error;
}

bool SettingsParser::reloadSettings()
{
    if (filepath_cache.empty())
    {
        last_error = "No filepath cached for reloading";
        return false;
    }
    return loadSettings(filepath_cache);
}

void SettingsParser::applyDefaultSettings()
{
    // Apply all the default values from the struct initializers
    // This ensures the system works even without a settings file

    settings = GangBeastsSettings::GangBeastsPhysicsSettings();
    settings.settings_loaded = false; // Mark as using defaults

    std::cout << "🎮 Using default Gang Beasts physics settings" << std::endl;
}

void SettingsParser::printSettings() const
{
    std::cout << "\n🎮 === GANG BEASTS PHYSICS SETTINGS ===" << std::endl;
    std::cout << "📦 Version: " << settings.version << std::endl;
    std::cout << "✅ Loaded: " << (settings.settings_loaded ? "From file" : "Defaults") << std::endl;

    std::cout << "\n🦴 Skeleton Springs:" << std::endl;
    std::cout << "  Base Stiffness: " << (settings.physics_personality.skeleton_springs.base_stiffness_multiplier * 100) << "%" << std::endl;
    std::cout << "  Arm Joints: " << (settings.physics_personality.skeleton_springs.arm_joints * 100) << "%" << std::endl;
    std::cout << "  Leg Joints: " << (settings.physics_personality.skeleton_springs.leg_joints * 100) << "%" << std::endl;

    std::cout << "\n⚖️ Postural Stability:" << std::endl;
    std::cout << "  Reaction Delay: " << (settings.postural_stability.reaction_timing.base_reaction_delay * 1000) << "ms" << std::endl;
    std::cout << "  Overcompensation: " << (settings.postural_stability.overcompensation.overcompensation_factor * 100) << "%" << std::endl;

    std::cout << "\n🔧 Advanced Spring System:" << std::endl;
    std::cout << "  Context-Aware Stiffness: " << (settings.advanced_spring_system.context_aware_stiffness.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Walking Stiffness: " << (settings.advanced_spring_system.context_aware_stiffness.walking_stiffness_multiplier * 100) << "%" << std::endl;
    std::cout << "  Jumping Stiffness: " << (settings.advanced_spring_system.context_aware_stiffness.jumping_stiffness_multiplier * 100) << "%" << std::endl;
    std::cout << "  Spring Animation System: " << (settings.advanced_spring_system.spring_animation_system.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Max Animations: " << settings.advanced_spring_system.spring_animation_system.max_concurrent_animations << std::endl;
    std::cout << "  Strength Modulation: " << (settings.advanced_spring_system.spring_strength_modulation.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Adaptive Physics: " << (settings.advanced_spring_system.adaptive_physics.enabled ? "Enabled" : "Disabled") << std::endl;

    /*
    std::cout << "\n🚶 Physics-Driven Walking:" << std::endl;
    std::cout << "  Walking Gait: " << (settings.physics_driven_walking.walking_gait.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Step Frequency: " << settings.physics_driven_walking.walking_gait.step_frequency << " Hz" << std::endl;
    std::cout << "  Center-of-Mass Dynamics: " << (settings.physics_driven_walking.center_of_mass_dynamics.enabled ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Dynamic Balancing: " << (settings.physics_driven_walking.center_of_mass_dynamics.dynamic_balancing ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Visual Enhancements: " << (settings.physics_driven_walking.visual_enhancements.enabled ? "Enabled" : "Disabled") << std::endl;
    */

    std::cout << "===================================\n"
              << std::endl;
}

// Phase 2: Physics-Driven Walking parser implementation
void SettingsParser::parsePhysicsDrivenWalking(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        if (line.find("\"walking_gait\"") != std::string::npos)
        {
            parseWalkingGait(file);
        }
        else if (line.find("\"center_of_mass_dynamics\"") != std::string::npos)
        {
            parseCenterOfMassDynamics(file);
        }
        else if (line.find("\"visual_enhancements\"") != std::string::npos)
        {
            parseVisualEnhancements(file);
        }
    }
}

void SettingsParser::parseWalkingGait(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.physics_driven_walking.walking_gait.enabled = parseBool(value, true);
            }
            else if (key == "step_cycle_duration")
            {
                settings.physics_driven_walking.walking_gait.step_cycle_duration = parseFloat(value, 1.0f);
            }
            else if (key == "step_height")
            {
                settings.physics_driven_walking.walking_gait.step_height = parseFloat(value, 25.0f);
            }
            else if (key == "step_length")
            {
                settings.physics_driven_walking.walking_gait.step_length = parseFloat(value, 40.0f);
            }
            else if (key == "step_wobble_amount")
            {
                settings.physics_driven_walking.walking_gait.step_wobble_amount = parseFloat(value, 0.1f);
            }
            else if (key == "lift_force_strength")
            {
                settings.physics_driven_walking.walking_gait.lift_force_strength = parseFloat(value, 35.0f); // PHYSICS FIX: Default fallback reduced
            }
            else if (key == "swing_force_strength")
            {
                settings.physics_driven_walking.walking_gait.swing_force_strength = parseFloat(value, 25.0f); // PHYSICS FIX: Default fallback reduced
            }
            else if (key == "plant_force_strength")
            {
                settings.physics_driven_walking.walking_gait.plant_force_strength = parseFloat(value, 50.0f); // PHYSICS FIX: Default fallback reduced
            }
        }
    }
}

void SettingsParser::parseCenterOfMassDynamics(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "enabled")
            {
                settings.physics_driven_walking.center_of_mass_dynamics.enabled = parseBool(value, true);
            }
            // Note: For now, we'll skip the nested structure fields that don't exist in current struct
            // These can be added when we expand the center of mass dynamics structure
        }
    }
}

void SettingsParser::parseVisualEnhancements(std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);

        if (line.find("}") != std::string::npos && line.find("\"") == std::string::npos)
        {
            break;
        }

        std::string key, value;
        if (parseLine(line, key, value))
        {
            if (key == "foot_contact_effects")
            {
                settings.physics_driven_walking.visual_enhancements.foot_contact_effects = parseBool(value, true);
            }
            else if (key == "ground_interaction")
            {
                settings.physics_driven_walking.visual_enhancements.ground_interaction = parseBool(value, true);
            }
            else if (key == "step_sound_triggers")
            {
                settings.physics_driven_walking.visual_enhancements.step_sound_triggers = parseBool(value, true);
            }
            else if (key == "weight_shift_visualization")
            {
                settings.physics_driven_walking.visual_enhancements.weight_shift_visualization = parseBool(value, false);
            }
            else if (key == "gait_debug_display")
            {
                settings.physics_driven_walking.visual_enhancements.gait_debug_display = parseBool(value, false);
            }
        }
    }
}
