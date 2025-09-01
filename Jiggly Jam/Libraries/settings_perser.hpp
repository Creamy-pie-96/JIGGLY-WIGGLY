#pragma once

#include "libs.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

// Simple JSON-like parser for Gang Beasts physics settings
// Handles nested objects and basic data types for our physics parameters

namespace GangBeastsSettings
{

    /**
     * 🎮 Gang Beasts Physics Settings Structure
     *
     * This struct contains all tunable parameters for Phase 1 of the Gang Beasts Evolution:
     * - Step 1.1: Physics Personality Tuning
     * - Step 1.2: Enhanced Postural Stability
     *
     * All values are easily adjustable via settings.json without recompilation.
     */
    struct PhysicsPersonality
    {
        // Step 1.1: Skeleton Spring Tuning
        struct SkeletonSprings
        {
            float base_stiffness_multiplier = 0.65f; // Reduces overall skeleton stiffness by 35%

            // Individual joint looseness factors (0.0 = loose, 1.0 = rigid)
            float spine_joints = 0.7f;  // Spine flexibility for body sway
            float arm_joints = 0.5f;    // Loose arms for Gang Beasts feel
            float leg_joints = 0.8f;    // Legs need some rigidity for walking
            float neck_joint = 0.6f;    // Wobbly head movement
            float wrist_joints = 0.4f;  // Very loose wrists for floppy hands
            float ankle_joints = 0.75f; // Ankle stability vs flexibility
        } skeleton_springs;

        // Step 1.1: Flesh Spring Wobbliness
        struct FleshSprings
        {
            float damping_multiplier = 1.4f;   // Increased damping for wobble
            float stiffness_reduction = 0.25f; // 25% reduction in flesh stiffness
            float wobble_frequency = 0.85f;    // Natural wobble frequency
            float wobble_amplitude = 1.2f;     // Wobble strength multiplier
        } flesh_springs;

        // Step 1.1: Mass Distribution for Top-Heavy Instability
        struct MassDistribution
        {
            float head_mass_multiplier = 1.3f;  // Heavier head for instability
            float torso_mass_multiplier = 1.1f; // Slightly heavier torso
            float limb_mass_multiplier = 0.9f;  // Lighter limbs for floppiness
        } mass_distribution;

        // Step 1.1: Spring Fatigue System
        struct SpringFatigue
        {
            bool enabled = true;         // Enable spring fatigue
            float fatigue_rate = 0.02f;  // How fast springs weaken
            float recovery_rate = 0.01f; // How fast they recover
            float min_strength = 0.6f;   // Minimum spring strength
            float max_strength = 1.0f;   // Maximum spring strength
        } spring_fatigue;
    };

    struct PosturalStability
    {
        // Step 1.2: Reaction Timing for Realism
        struct ReactionTiming
        {
            float base_reaction_delay = 0.0f;   // 0.0 = No delay, original instant response
            float panic_reaction_delay = 0.0f;  // Faster reactions when panicking
            float recovery_acceleration = 1.0f; // How fast recovery speeds up
        } reaction_timing;

        // Step 1.2: Overcompensation for Comedy
        struct Overcompensation
        {
            bool enabled = false;                 // Disabled by default (original behavior)
            float overcompensation_factor = 1.0f; // 1.0 = original balance strength
            float oscillation_damping = 1.0f;     // Damping for balance oscillations
            float wobble_decay_time = 0.0f;       // Time for wobbles to settle
        } overcompensation;

        // Step 1.2: Sensitivity Parameters
        struct Sensitivity
        {
            float balance_threshold = 0.12f;         // When to start balance corrections
            float panic_threshold = 0.25f;           // When to panic and overreact
            float recovery_threshold = 0.05f;        // When balance is considered recovered
            float ground_detection_tolerance = 8.0f; // Ground contact detection range
        } sensitivity;

        // Step 1.2: Comedy Modes for Testing
        struct ComedyModes
        {
            struct DrunkWalking
            {
                bool enabled = false;                   // Disabled by default
                float sway_amplitude = 0.0f;            // No sway by default
                float sway_frequency = 0.0f;            // No sway frequency
                float random_stumble_chance = 0.0f;     // No stumbling by default
                float reaction_delay_multiplier = 1.0f; // Original reaction speed
            } drunk_walking;

            struct NervousCharacter
            {
                bool enabled = false;                 // Disabled by default
                float overreaction_multiplier = 1.0f; // Original reaction strength
                float fidget_frequency = 0.0f;        // No fidgeting by default
                float balance_sensitivity = 1.0f;     // Original sensitivity
            } nervous_character;

            struct ConfidentCharacter
            {
                bool enabled = false;                  // Disabled by default
                float stability_confidence = 1.0f;     // Original balance
                float reaction_delay_reduction = 1.0f; // Original reaction speed
                float swagger_amplitude = 0.0f;        // No swagger by default
            } confident_character;
        } comedy_modes;
    };

    // Step 1.3: Advanced Spring System
    struct AdvancedSpringSystem
    {
        // Context-aware spring behavior
        struct ContextAwareStiffness
        {
            bool enabled = true;                        // System enabled but no changes applied
            float walking_stiffness_multiplier = 1.0f;  // Original walking stiffness
            float jumping_stiffness_multiplier = 1.0f;  // Original jumping stiffness
            float falling_stiffness_multiplier = 1.0f;  // Original falling stiffness
            float grabbing_stiffness_multiplier = 1.0f; // Original grabbing stiffness
            float idle_stiffness_multiplier = 1.0f;     // Original idle stiffness
            float transition_speed = 2.0f;              // How fast stiffness changes
        } context_aware_stiffness;

        // Keyframe-based spring animation
        struct SpringAnimationSystem
        {
            bool enabled = true;                           // System enabled
            int max_concurrent_animations = 8;             // Max simultaneous animations
            float animation_blend_time = 0.3f;             // Blend time between animations
            std::string keyframe_interpolation = "smooth"; // Interpolation type

            struct PrioritySystem
            {
                int walking_priority = 3;
                int jumping_priority = 5;
                int falling_priority = 7;
                int grabbing_priority = 6;
                int waving_priority = 2;
                int idle_priority = 1;
            } priority_system;
        } spring_animation_system;

        // Dynamic spring strength modulation
        struct SpringStrengthModulation
        {
            bool enabled = true; // System enabled

            struct ActionStrengthFactors
            {
                float walking_leg_strength = 1.0f;    // Original walking leg strength
                float walking_arm_strength = 1.0f;    // Original walking arm strength
                float jumping_leg_strength = 1.0f;    // Original jumping leg strength
                float landing_leg_strength = 1.0f;    // Original landing leg strength
                float grabbing_arm_strength = 1.0f;   // Original grabbing arm strength
                float waving_arm_strength = 1.0f;     // Original waving arm strength
                float balancing_core_strength = 1.0f; // Original balancing core strength
            } action_strength_factors;

            float modulation_smoothing = 3.0f; // Smoothness of strength changes
        } spring_strength_modulation;

        // Adaptive physics system
        struct AdaptivePhysics
        {
            bool enabled = true;            // System enabled
            float learning_rate = 0.0f;     // No adaptation by default
            float adaptation_memory = 5.0f; // How long adaptations last

            struct StressResponse
            {
                bool enabled = false;          // Disabled by default
                float stress_threshold = 2.0f; // When springs enter stress
                float recovery_time = 3.0f;    // Time to recover from stress
                float stress_weakening = 0.0f; // No weakening by default
            } stress_response;
        } adaptive_physics;
    };

    struct PerformanceSettings
    {
        int physics_substeps = 3;                 // Physics simulation substeps
        int stability_update_frequency = 60;      // Hz for stability updates
        float spring_animation_precision = 0.01f; // Precision for spring animations
        float force_calculation_threshold = 0.1f; // Minimum force to calculate
    };

    struct DebugSettings
    {
        bool show_stability_forces = false;  // Visualize stability forces
        bool show_spring_stress = false;     // Show spring stress colors
        bool show_mass_distribution = false; // Visualize mass distribution
        bool log_physics_events = false;     // Log physics events to console
        bool performance_profiling = false;  // Enable performance profiling
    };

    /**
     * 🎮 Complete Gang Beasts Settings Container
     *
     * Contains all settings for Gang Beasts physics evolution.
     * Loaded from settings.json and accessible throughout the physics system.
     */
    struct GangBeastsPhysicsSettings
    {
        PhysicsPersonality physics_personality;
        PosturalStability postural_stability;
        AdvancedSpringSystem advanced_spring_system; // Step 1.3: Advanced Spring System
        PerformanceSettings performance;
        DebugSettings debug;

        std::string version = "1.0";
        bool settings_loaded = false;
    };

} // namespace GangBeastsSettings

/**
 * 🎮 Gang Beasts Settings Parser
 *
 * Loads and parses settings.json file for Gang Beasts physics parameters.
 * Provides simple interface to access all tunable parameters.
 *
 * Usage:
 *   SettingsParser parser;
 *   if (parser.loadSettings("Game Settings/settings.json")) {
 *       auto settings = parser.getSettings();
 *       // Use settings.physics_personality.skeleton_springs.base_stiffness_multiplier
 *   }
 */
class SettingsParser
{
private:
    GangBeastsSettings::GangBeastsPhysicsSettings settings;
    std::string last_error;

    // Simple JSON-like parser helpers
    std::string trim(const std::string &str);
    bool parseLine(const std::string &line, std::string &key, std::string &value);
    float parseFloat(const std::string &value, float defaultValue = 0.0f);
    bool parseBool(const std::string &value, bool defaultValue = false);
    int parseInt(const std::string &value, int defaultValue = 0);

    // Specific section parsers
    void parsePhysicsPersonality(std::ifstream &file);
    void parsePosturalStability(std::ifstream &file);
    void parseAdvancedSpringSystem(std::ifstream &file); // Step 1.3: Parser for advanced spring system
    void parsePerformanceSettings(std::ifstream &file);
    void parseDebugSettings(std::ifstream &file);

    // Nested object parsers
    void parseSkeletonSprings(std::ifstream &file);
    void parseFleshSprings(std::ifstream &file);
    void parseMassDistribution(std::ifstream &file);
    void parseSpringFatigue(std::ifstream &file);
    void parseReactionTiming(std::ifstream &file);
    void parseOvercompensation(std::ifstream &file);
    void parseSensitivity(std::ifstream &file);
    void parseComedyModes(std::ifstream &file);

    // Step 1.3: Advanced Spring System nested parsers
    void parseContextAwareStiffness(std::ifstream &file);
    void parseSpringAnimationSystem(std::ifstream &file);
    void parseSpringStrengthModulation(std::ifstream &file);
    void parseAdaptivePhysics(std::ifstream &file);

public:
    SettingsParser();
    ~SettingsParser();

    /**
     * Load settings from JSON file
     * @param filepath Path to settings.json file
     * @return true if loaded successfully, false otherwise
     */
    bool loadSettings(const std::string &filepath);

    /**
     * Get the loaded settings
     * @return Reference to the settings structure
     */
    const GangBeastsSettings::GangBeastsPhysicsSettings &getSettings() const;

    /**
     * Check if settings were loaded successfully
     * @return true if settings are loaded and valid
     */
    bool isLoaded() const;

    /**
     * Get last error message
     * @return Error message string
     */
    const std::string &getLastError() const;

    /**
     * Reload settings from the same file
     * @return true if reloaded successfully
     */
    bool reloadSettings();

    /**
     * Save current settings back to file (for runtime adjustments)
     * @param filepath Path where to save settings
     * @return true if saved successfully
     */
    bool saveSettings(const std::string &filepath) const;

    /**
     * Apply default settings (fallback if file loading fails)
     */
    void applyDefaultSettings();

    /**
     * Print current settings to console (for debugging)
     */
    void printSettings() const;

private:
    std::string filepath_cache; // For reloading
};
