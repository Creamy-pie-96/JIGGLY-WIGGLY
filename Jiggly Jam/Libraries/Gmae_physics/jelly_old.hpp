#pragma once
#include "../libs.hpp"

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

    void add_point(const sf::Vector2f &pos, BODY_PART body_part)
    {
        // Game-physics based mass distribution for proper balance
        float mass = 1.0f; // default

        // Heavy skeletal structure for stability
        if (body_part == BODY_PART::PELVIS || body_part == BODY_PART::SPINE_LOW ||
            body_part == BODY_PART::SPINE_MID || body_part == BODY_PART::SPINE_UP ||
            body_part == BODY_PART::HEAD)
        {
            mass = 5.0f; // Heavy core/spine
        }
        // Very heavy feet for grounding
        else if (body_part == BODY_PART::FOOT_L || body_part == BODY_PART::FOOT_R ||
                 body_part == BODY_PART::ANKLE_L || body_part == BODY_PART::ANKLE_R)
        {
            mass = 8.0f; // Very heavy feet for ground contact
        }
        // Medium weight limbs
        else if (body_part == BODY_PART::HIP_L || body_part == BODY_PART::HIP_R ||
                 body_part == BODY_PART::KNEE_L || body_part == BODY_PART::KNEE_R ||
                 body_part == BODY_PART::SHO_L || body_part == BODY_PART::SHO_R ||
                 body_part == BODY_PART::ELB_L || body_part == BODY_PART::ELB_R ||
                 body_part == BODY_PART::WRIST_L || body_part == BODY_PART::WRIST_R)
        {
            mass = 2.0f; // Medium limb joints
        }
        // Light flesh/tissue
        else if (body_part == BODY_PART::NONE)
        {
            mass = 0.5f; // Light soft tissue
        }

        points.push_back({pos, pos, {0.f, 0.f}, false, mass, body_part, id});
        part_index[body_part] = points.size() - 1;
    }

    void add_edge(BODY_PART a, BODY_PART b)
    {
        if (part_index.find(a) == part_index.end() || part_index.find(b) == part_index.end())
            return;
        int ia = part_index[a];
        int ib = part_index[b];
        if (ia == ib)
            return;
        sf::Vector2f d = points[ib].pos - points[ia].pos;
        float dist = std::sqrt(d.x * d.x + d.y * d.y);
        if (dist < 1e-3f)
            dist = 1e-3f;

        // Use default skeleton stiffness - hierarchy is handled elsewhere
        springs.push_back({ia, ib, dist, skeletonStiffness, true, id});
    }

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

    float get_radius() const { return radius; }

    Jelly(int n, float r, sf::Vector2f c);
};

// Helper function for unique ID generation
uint64_t generateID();

#pragma once
#include "../libs.hpp"

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

    void add_point(const sf::Vector2f &pos, BODY_PART body_part)
    {
        // Game-physics based mass distribution for proper balance
        float mass = 1.0f; // default

        // Heavy skeletal structure for stability
        if (body_part == BODY_PART::PELVIS || body_part == BODY_PART::SPINE_LOW ||
            body_part == BODY_PART::SPINE_MID || body_part == BODY_PART::SPINE_UP ||
            body_part == BODY_PART::HEAD)
        {
            mass = 5.0f; // Heavy core/spine
        }
        // Very heavy feet for grounding
        else if (body_part == BODY_PART::FOOT_L || body_part == BODY_PART::FOOT_R ||
                 body_part == BODY_PART::ANKLE_L || body_part == BODY_PART::ANKLE_R)
        {
            mass = 8.0f; // Very heavy feet for ground contact
        }
        // Medium weight limbs
        else if (body_part == BODY_PART::HIP_L || body_part == BODY_PART::HIP_R ||
                 body_part == BODY_PART::KNEE_L || body_part == BODY_PART::KNEE_R ||
                 body_part == BODY_PART::SHO_L || body_part == BODY_PART::SHO_R ||
                 body_part == BODY_PART::ELB_L || body_part == BODY_PART::ELB_R ||
                 body_part == BODY_PART::WRIST_L || body_part == BODY_PART::WRIST_R)
        {
            mass = 2.0f; // Medium limb joints
        }
        // Light flesh/tissue
        else if (body_part == BODY_PART::NONE)
        {
            mass = 0.5f; // Light soft tissue
        }

        points.push_back({pos, pos, {0.f, 0.f}, false, mass, body_part, id});
        part_index[body_part] = points.size() - 1;
    }

    void add_edge(BODY_PART a, BODY_PART b)
    {
        if (part_index.find(a) == part_index.end() || part_index.find(b) == part_index.end())
            return;
        int ia = part_index[a];
        int ib = part_index[b];
        if (ia == ib)
            return;
        sf::Vector2f d = points[ib].pos - points[ia].pos;
        float dist = std::sqrt(d.x * d.x + d.y * d.y);
        if (dist < 1e-3f)
            dist = 1e-3f;

        // Use default skeleton stiffness - hierarchy is handled elsewhere
        springs.push_back({ia, ib, dist, skeletonStiffness, true, id});
    }

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

    float get_radius() const { return radius; }

    Jelly(int n, float r, sf::Vector2f c);
};

// Helper function for unique ID generation
uint64_t generateID();

