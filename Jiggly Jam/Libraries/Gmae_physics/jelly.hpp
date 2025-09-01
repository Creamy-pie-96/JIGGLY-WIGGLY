#pragma once
#include "../libs.hpp"
#include "../settings_perser.hpp"

enum class FORCE_TYPE
{
    GLOBAL_FORCE,
    LOCAL_FORCE,
    IMPULSE_FORCE
};
enum class BODY_PART
{
    // Core skeleton joints (canonical 20-joint rig)
    HEAD,
    NECK,
    SPINE_UP,  // Upper spine/chest
    SPINE_MID, // Middle spine
    SPINE_LOW, // Lower spine/pelvis connection
    CLAV_R,    // Right clavicle
    CLAV_L,    // Left clavicle
    SHO_R,     // Right Shoulder
    ELB_R,     // Right Elbow
    WRIST_R,   // Right Wrist
    HAND_R,    // Right Hand
    SHO_L,     // Left Shoulder
    ELB_L,     // Left Elbow
    WRIST_L,   // Left Wrist
    HAND_L,    // Left Hand
    PELVIS,    // Central pelvis/waist
    HIP_L,     // Left Hip
    KNEE_L,    // Left Knee
    ANKLE_L,   // Left Ankle
    FOOT_L,    // Left Foot
    HIP_R,     // Right Hip
    KNEE_R,    // Right Knee
    ANKLE_R,   // Right Ankle
    FOOT_R,    // Right Foot
    NONE       // Flesh points (not skeleton)
};
struct Point
{
    sf::Vector2f pos, prev_pos, acc = {0.f, 0.f};

    bool locked = false;
    float mass = 0.0001f;
    BODY_PART body_part;
    uint64_t id;
};
struct Spring
{
    int p1, p2;
    float rest_length;
    // per-spring stiffness multiplier (0..1) to allow stronger skeleton springs
    float stiffness = 1.f;
    // mark if this spring is part of skeleton wiring
    bool is_skeleton = false;
    // owner id (player id) for skeleton springs, 0 = none
    uint64_t owner_id = 0;
};

class Jelly
{
private:
    int N = 20;
    float radius = 30.f;
    sf::Vector2f center{200.f, 300.f};
    void update_verlet(float dt, sf::Vector2f acceleration = {0, 0});

public:
    Jelly();
    ~Jelly();

    std::vector<Point> points;
    std::vector<Spring> springs;
    int iterations = 25; // Increased for human figure stability
    // global stiffness multiplier for constraint correction (0..1)
    float stiffness = 0.8f; // Stronger for human figures
    // separate multipliers
    float stiffness_ring = 0.6f;   // peripheral-peripheral springs
    float stiffness_spoke = 0.65f; // center-peripheral springs
    // Spring hierarchy for proper body mechanics
    float stiffness_bone = 0.95f; // Skeleton/bone springs (very rigid)
    float stiffness_joint = 0.8f; // Major joints (rigid but flexible)
    float stiffness_flesh = 0.3f; // Soft tissue (flexible)
    // internal pressure (0 = off). Positive -> expansion force
    float pressure = 0.5f; // Reduced to avoid conflict with grounding
    // damping multiplier applied to velocity (0..1, closer to 1 more bouncy)
    float damping = 0.96f; // Slightly more damping for stability
    // target area for pressure preservation
    float targetArea = 0.f;
    void update(float dt);
    void apply_force(const sf::Vector2f &F_total);
    void apply_force(const sf::Vector2f &F_total, const sf::Vector2f &at, float radius);
    void apply_force(const sf::Vector2f &J_total, const sf::Vector2f &at, float radius, float dt);
    void draw(sf::RenderWindow &window);

    void add_point(const sf::Vector2f &pos, BODY_PART body_part);
    void add_edge(BODY_PART a, BODY_PART b);

    // shape creation helpers
    void clear();
    void create_circle(int n, float r, sf::Vector2f c);
    void create_from_points(const std::vector<sf::Vector2f> &pts);
    void create_from_points(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts);
    // create from points but resample edges to have approximately target peripheral points
    void create_from_points_resampled(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts, int targetPerimeterPoints, uint64_t id);
    // create a filled soft-body by sampling the interior of a polygon with a triangular lattice
    // polygon: ordered peripheral points
    // spacing: approximate distance between interior lattice points (pixels)
    void create_filled_from_polygon(const std::vector<sf::Vector2f> &pts, float spacing = 12.f);
    // overload: accept perimeter points paired with BODY_PART tags and an owner id
    void create_filled_from_polygon(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts, float spacing, uint64_t id);
    // add skeleton springs for a given player id by wiring BODY_PART tagged perimeter points
    void add_skeleton_for_player(uint64_t id, float skeletonStiffness = 1.5f);
    // connect interior/flesh points to nearest skeleton joints for soft attachment
    void connect_flesh_to_skeleton(uint64_t owner_id, float fleshToSkeletonStiffness = 0.4f, int k = 2, float maxDistance = 60.f);
    // apply postural stability forces to maintain upright stance
    void apply_postural_stability(uint64_t owner_id, float dt, float ground_y = 400.f);
    // control helpers for body parts
    int find_part_index(uint64_t owner_id, BODY_PART part) const;
    void apply_impulse_to_part(uint64_t owner_id, BODY_PART part, const sf::Vector2f &impulse);
    void set_part_target(uint64_t owner_id, BODY_PART part, const sf::Vector2f &targetPos, float kp = 150.f, float kd = 10.f);
    void clear_part_target(uint64_t owner_id, BODY_PART part);
    void animate_skeleton_spring(uint64_t owner_id, BODY_PART a, BODY_PART b, float newRest, float lerp = 1.f);

    // target store for PD controllers
    struct PartTarget
    {
        uint64_t owner = 0;
        BODY_PART part = BODY_PART::NONE;
        sf::Vector2f target{0.f, 0.f};
        float kp = 0.f;
        float kd = 0.f;
        bool active = false;
    };
    std::vector<PartTarget> partTargets;

    // per-jelly owner id (used when tagging points/springs for players)
    uint64_t id = 0;
    // default stiffness for skeleton edges added by add_edge
    float skeletonStiffness = 1.5f;
    // map BODY_PART -> point index for helper wiring
    std::unordered_map<BODY_PART, int> part_index;

    // 🎮 GANG BEASTS EVOLUTION: Settings Integration
    const GangBeastsSettings::GangBeastsPhysicsSettings *gangBeastsSettings = nullptr;

    // Step 1.1: Physics Personality members
    struct SpringFatigueState
    {
        float current_strength = 1.0f;
        float fatigue_accumulation = 0.0f;
        bool is_fatigued = false;
    };
    std::unordered_map<int, SpringFatigueState> springFatigueStates; // spring index -> fatigue state

    // Step 1.2: Enhanced Postural Stability members
    struct PosturalState
    {
        float reaction_timer = 0.0f;                 // Delay before stability kicks in
        float imbalance_magnitude = 0.0f;            // Current imbalance level
        bool is_panicking = false;                   // In panic mode (stronger reactions)
        sf::Vector2f stability_momentum{0.0f, 0.0f}; // Accumulated stability forces
        float wobble_decay_timer = 0.0f;             // Timer for wobble decay
    };
    std::unordered_map<uint64_t, PosturalState> posturalStates; // owner_id -> postural state

    float get_radius() const { return radius; }

    // 🎮 GANG BEASTS EVOLUTION: Settings Management
    void setGangBeastsSettings(const GangBeastsSettings::GangBeastsPhysicsSettings *settings);
    bool hasGangBeastsSettings() const { return gangBeastsSettings != nullptr; }

    // Step 1.1: Physics Personality Methods
    float getSkeletonStiffnessMultiplier(BODY_PART partA, BODY_PART partB) const;
    float getFleshDampingMultiplier() const;
    float getMassMultiplier(BODY_PART bodyPart) const;
    void updateSpringFatigue(float dt);
    void applySpringFatigue(int springIndex, float &stiffness);

    // Step 1.2: Enhanced Postural Stability Methods
    void apply_enhanced_postural_stability(uint64_t owner_id, float dt, float ground_y = 400.f);
    float calculateImbalanceMagnitude(uint64_t owner_id, float ground_y);
    void applyReactionDelay(uint64_t owner_id, float dt);
    void applyOvercompensation(uint64_t owner_id, sf::Vector2f &stabilityForce);
    bool shouldEnterPanicMode(uint64_t owner_id);

    // Step 1.3: Advanced Spring System members
    enum class PhysicsState
    {
        IDLE,
        WALKING,
        JUMPING,
        FALLING,
        GRABBING,
        WAVING
    };

    struct AdvancedSpringState
    {
        PhysicsState current_state = PhysicsState::IDLE;
        PhysicsState previous_state = PhysicsState::IDLE;
        float transition_timer = 0.0f; // For smooth transitions
        float state_duration = 0.0f;   // How long in current state

        // Context-aware stiffness
        float current_stiffness_multiplier = 1.0f;
        float target_stiffness_multiplier = 1.0f;

        // Spring animation system
        struct ActiveAnimation
        {
            std::string name;
            float progress = 0.0f; // 0.0 to 1.0
            float duration = 1.0f;
            int priority = 0;
            bool is_looping = false;
        };
        std::vector<ActiveAnimation> active_animations;

        // Dynamic strength modulation per spring type
        std::unordered_map<int, float> spring_strength_overrides; // spring index -> strength multiplier

        // Adaptive physics
        struct AdaptiveState
        {
            float stress_level = 0.0f;
            float accumulated_forces = 0.0f;
            float last_adaptation_time = 0.0f;
            std::unordered_map<int, float> learned_stiffness; // spring index -> learned stiffness
        } adaptive_state;
    };
    std::unordered_map<uint64_t, AdvancedSpringState> advancedSpringStates; // owner_id -> spring state

    // Step 1.3: Advanced Spring System Methods
    void updateAdvancedSpringSystems(float dt);
    void updateContextAwareStiffness(uint64_t owner_id, float dt);
    void updateSpringAnimationSystem(uint64_t owner_id, float dt);
    void updateSpringStrengthModulation(uint64_t owner_id, float dt);
    void updateAdaptivePhysics(uint64_t owner_id, float dt);

    // State detection methods
    PhysicsState detectPhysicsState(uint64_t owner_id) const;
    bool isWalking(uint64_t owner_id) const;
    bool isJumping(uint64_t owner_id) const;
    bool isFalling(uint64_t owner_id) const;
    bool isGrabbing(uint64_t owner_id) const;

    // Animation system methods
    void startSpringAnimation(uint64_t owner_id, const std::string &animationName, float duration, int priority, bool looping = false);
    void stopSpringAnimation(uint64_t owner_id, const std::string &animationName);
    float getAnimationStrengthMultiplier(uint64_t owner_id, const std::string &bodyRegion) const;

    // Adaptive physics methods
    void applyAdaptivePhysics(uint64_t owner_id, int springIndex, float &stiffness);
    void recordSpringStress(uint64_t owner_id, int springIndex, float stress);

    Jelly(int n, float r, sf::Vector2f c);
};

// Helper function for unique ID generation
uint64_t generateID();
