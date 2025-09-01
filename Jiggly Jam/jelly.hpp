#pragma once
#include "libs.hpp"

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

// default delegates to param constructor
Jelly::Jelly() : Jelly(20, 30.f, {200.f, 300.f}) {}

Jelly::~Jelly() {}

Jelly::Jelly(int n, float r, sf::Vector2f c) : N(n), radius(r), center(c)
{
    create_circle(N, radius, center);
}

int Jelly::find_part_index(uint64_t owner_id, BODY_PART part) const
{
    // find first perimeter point matching owner and body_part
    for (int i = 1; i <= N && i < (int)points.size(); ++i)
    {
        if (points[i].id == owner_id && points[i].body_part == part)
            return i;
    }
    // fallback: search any point (interior) with owner id
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].id == owner_id && points[i].body_part == part)
            return (int)i;
    }
    return -1;
}

void Jelly::apply_impulse_to_part(uint64_t owner_id, BODY_PART part, const sf::Vector2f &impulse)
{
    int idx = find_part_index(owner_id, part);
    if (idx <= 0 || idx >= (int)points.size())
        return;
    Point &p = points[idx];
    // impulse -> change in velocity: dv = impulse / m
    sf::Vector2f dv = impulse / p.mass;
    // implement by shifting prev_pos so Verlet integrates with added velocity
    p.prev_pos -= dv;
}

void Jelly::set_part_target(uint64_t owner_id, BODY_PART part, const sf::Vector2f &targetPos, float kp, float kd)
{
    // find or create target entry
    for (auto &t : partTargets)
    {
        if (t.owner == owner_id && t.part == part)
        {
            t.target = targetPos;
            t.kp = kp;
            t.kd = kd;
            t.active = true;
            return;
        }
    }
    PartTarget nt;
    nt.owner = owner_id;
    nt.part = part;
    nt.target = targetPos;
    nt.kp = kp;
    nt.kd = kd;
    nt.active = true;
    partTargets.push_back(nt);
}

void Jelly::clear_part_target(uint64_t owner_id, BODY_PART part)
{
    for (auto &t : partTargets)
    {
        if (t.owner == owner_id && t.part == part)
        {
            t.active = false;
            return;
        }
    }
}

void Jelly::animate_skeleton_spring(uint64_t owner_id, BODY_PART a, BODY_PART b, float newRest, float lerp)
{
    // find skeleton spring connecting parts for this owner and adjust rest_length
    int ia = find_part_index(owner_id, a);
    int ib = find_part_index(owner_id, b);
    if (ia <= 0 || ib <= 0)
        return;
    for (auto &s : springs)
    {
        if (!s.is_skeleton || s.owner_id != owner_id)
            continue;
        if ((s.p1 == ia && s.p2 == ib) || (s.p1 == ib && s.p2 == ia))
        {
            if (lerp >= 1.f)
                s.rest_length = newRest;
            else
                s.rest_length = s.rest_length * (1.f - lerp) + newRest * lerp;
            return;
        }
    }
}

void Jelly::clear()
{
    points.clear();
    springs.clear();
}

void Jelly::create_circle(int n, float r, sf::Vector2f c)
{
    N = std::max(3, n);
    radius = r;
    center = c;
    clear();

    // center at index 0
    points.resize(N + 1);
    points[0].pos = center;
    points[0].prev_pos = center;
    points[0].locked = false;
    points[0].mass = 1.f;
    points[0].body_part = BODY_PART::NONE;

    for (int i = 1; i <= N; ++i)
    {
        float angle = i * 2.f * M_PI / float(N);
        sf::Vector2f p = center + sf::Vector2f(std::cos(angle), std::sin(angle)) * radius;
        points[i].pos = p;
        points[i].prev_pos = p;
        points[i].mass = 1.f;
        points[i].locked = false;
        points[i].body_part = BODY_PART::NONE;
    }

    // create springs: ring + center spokes
    springs.clear();
    int centerIndex = 0;
    for (int i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1; // wraps to 1
        sf::Vector2f delta = points[next].pos - points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({i, next, dist, stiffness_ring, false, 0});

        sf::Vector2f delta2 = points[i].pos - points[centerIndex].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({centerIndex, i, dist2, stiffness_spoke, false, 0});
    }
}

// compute polygon area for the first `perimCount` peripheral points (indices 1..perimCount)
static float compute_area(const std::vector<Point> &points, int perimCount)
{
    int n = perimCount;
    if (n < 3)
        return 0.f;
    float a = 0.f;
    for (int i = 1; i <= n; ++i)
    {
        int j = (i % n) + 1;
        a += points[i].pos.x * points[j].pos.y - points[j].pos.x * points[i].pos.y;
    }
    return 0.5f * std::abs(a);
}

// create from arbitrary list of peripheral points (ordered). We compute centroid as center and add spokes.
void Jelly::create_from_points(const std::vector<sf::Vector2f> &pts)
{
    if (pts.empty())
        return;
    // compute centroid
    sf::Vector2f c{0.f, 0.f};
    for (auto &p : pts)
        c += p;
    c /= float(pts.size());

    N = (int)pts.size();
    radius = 0.f;
    center = c;
    clear();

    points.resize(N + 1);
    points[0].pos = center;
    points[0].prev_pos = center;
    points[0].mass = 1.f;
    points[0].locked = false;

    for (int i = 1; i <= N; ++i)
    {
        points[i].pos = pts[i - 1];
        points[i].prev_pos = pts[i - 1];
        points[i].mass = 1.f;
        points[i].locked = false;
        float d = std::sqrt((points[i].pos.x - center.x) * (points[i].pos.x - center.x) + (points[i].pos.y - center.y) * (points[i].pos.y - center.y));
        radius = std::max(radius, d);
    }

    // springs
    springs.clear();
    for (int i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1;
        sf::Vector2f delta = points[next].pos - points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({i, next, dist, stiffness_ring, false, 0});

        sf::Vector2f delta2 = points[i].pos - points[0].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({0, i, dist2, stiffness_spoke, false, 0});
    }
    // compute and store target area for pressure (perimeter only)
    targetArea = compute_area(points, N);
}

void Jelly::create_from_points_resampled(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts, int targetPerimeterPoints, uint64_t id)
{
    if (pts.size() < 3)
    {
        create_from_points(pts);
        return;
    }

    // compute edge lengths and total
    int m = (int)pts.size();
    std::vector<float> edgeLen(m);
    float total = 0.f;
    for (int i = 0; i < m; ++i)
    {
        int j = (i + 1) % m;
        sf::Vector2f d = pts[j].first - pts[i].first;
        edgeLen[i] = std::sqrt(d.x * d.x + d.y * d.y);
        total += edgeLen[i];
    }

    // allocate points proportionally on edges
    std::vector<sf::Vector2f> res;
    for (int i = 0; i < m; ++i)
    {
        int j = (i + 1) % m;
        float portion = edgeLen[i] / total;
        int count = std::max(1, (int)std::round(portion * targetPerimeterPoints));
        for (int k = 0; k < count; ++k)
        {
            float t = float(k) / float(count);
            res.push_back(pts[i].first + (pts[j].first - pts[i].first) * t);
        }
    }

    // ensure we have at least targetPerimeterPoints
    while ((int)res.size() < targetPerimeterPoints)
    {
        res.push_back(pts[0].first);
    }

    create_from_points(res);
}

// helper: point-in-polygon (winding / raycast)
static bool point_in_polygon(const sf::Vector2f &pt, const std::vector<sf::Vector2f> &poly)
{
    bool inside = false;
    int n = (int)poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const sf::Vector2f &pi = poly[i];
        const sf::Vector2f &pj = poly[j];
        bool intersect = ((pi.y > pt.y) != (pj.y > pt.y)) && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y + 1e-12f) + pi.x);
        if (intersect)
            inside = !inside;
    }
    return inside;
}

void Jelly::create_filled_from_polygon(const std::vector<sf::Vector2f> &pts, float spacing)
{
    if (pts.size() < 3)
    {
        create_from_points(pts);
        return;
    }

    // compute centroid for index 0
    sf::Vector2f c{0.f, 0.f};
    for (auto &p : pts)
        c += p;
    c /= float(pts.size());

    // prepare perimeter as before
    int Nperim = (int)pts.size();
    points.clear();
    springs.clear();

    // reserve an estimated number of points
    points.reserve(Nperim + 64);

    points.resize(Nperim + 1);
    points[0].pos = c;
    points[0].prev_pos = c;
    points[0].mass = 1.f;
    points[0].locked = false;

    float maxr = 0.f;
    for (int i = 1; i <= Nperim; ++i)
    {
        points[i].pos = pts[i - 1];
        points[i].prev_pos = pts[i - 1];
        points[i].mass = 1.f;
        points[i].locked = false;
        float d = std::hypot(points[i].pos.x - c.x, points[i].pos.y - c.y);
        if (d > maxr)
            maxr = d;
    }

    // create perimeter ring + spokes
    for (int i = 1; i <= Nperim; ++i)
    {
        int next = (i % Nperim) + 1;
        sf::Vector2f delta = points[next].pos - points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({i, next, dist, stiffness_ring, false, 0});

        sf::Vector2f delta2 = points[i].pos - points[0].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({0, i, dist2, stiffness_spoke, false, 0});
    }

    // bounding box for sampling
    float xmin = pts[0].x, xmax = pts[0].x, ymin = pts[0].y, ymax = pts[0].y;
    for (auto &p : pts)
    {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }

    // triangular lattice steps (centered on polygon centroid to reduce sampling bias)
    float dx = spacing;
    float dy = spacing * 0.86602540378f; // sqrt(3)/2

    // compute sampling grid size to cover bounding box, add margin
    float width = xmax - xmin;
    float height = ymax - ymin;
    int cols = std::max(3, int(std::ceil(width / dx)) + 5);
    int rows = std::max(3, int(std::ceil(height / dy)) + 5);

    // origin so the grid is centered on centroid 'c'
    float origin_x = c.x - (cols - 1) * 0.5f * dx;
    float origin_y = c.y - (rows - 1) * 0.5f * dy;

    // map from grid coords to point index
    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, -1));

    // add interior points that lie inside polygon
    for (int r = 0; r < rows; ++r)
    {
        float y = origin_y + r * dy;
        float xoff = (r % 2) ? dx * 0.5f : 0.f;
        for (int cidx = 0; cidx < cols; ++cidx)
        {
            float x = origin_x + cidx * dx + xoff;
            sf::Vector2f pos{x, y};
            if (!point_in_polygon(pos, pts))
                continue;
            Point p;
            p.pos = pos;
            p.prev_pos = pos;
            p.mass = 1.f;
            p.locked = false;
            int idx = (int)points.size();
            points.push_back(p);
            grid[r][cidx] = idx;
        }
    }

    // create lattice springs connecting neighbors in triangular grid
    for (int r = 0; r < rows; ++r)
    {
        for (int cidx = 0; cidx < cols; ++cidx)
        {
            int idx = grid[r][cidx];
            if (idx == -1)
                continue;
            // neighbor offsets: right, down-right, down-left
            const int offs[3][2] = {{0, 1}, {1, (r % 2) ? 1 : 0}, {1, (r % 2) ? 0 : -1}};
            // simple explicit neighbors covering the triangular connectivity
            // right neighbor
            if (cidx + 1 < cols)
            {
                int j = grid[r][cidx + 1];
                if (j != -1)
                {
                    sf::Vector2f d = points[j].pos - points[idx].pos;
                    springs.push_back({idx, j, std::sqrt(d.x * d.x + d.y * d.y), stiffness_ring, false, 0});
                }
            }
            // down-right / up-right depending on row parity
            if (r + 1 < rows)
            {
                int cj = cidx + (r % 2 ? 0 : 1);
                if (cj >= 0 && cj < cols)
                {
                    int j = grid[r + 1][cj];
                    if (j != -1)
                    {
                        sf::Vector2f d = points[j].pos - points[idx].pos;
                        springs.push_back({idx, j, std::sqrt(d.x * d.x + d.y * d.y), stiffness_ring, false, 0});
                    }
                }
                int cj2 = cidx + (r % 2 ? 1 : 0);
                if (cj2 >= 0 && cj2 < cols)
                {
                    int j2 = grid[r + 1][cj2];
                    if (j2 != -1)
                    {
                        sf::Vector2f d = points[j2].pos - points[idx].pos;
                        springs.push_back({idx, j2, std::sqrt(d.x * d.x + d.y * d.y), stiffness_ring, false, 0});
                    }
                }
            }
        }
    }

    // connect nearby perimeter points to lattice boundary points so the border is glued
    for (int pi = 1; pi <= Nperim; ++pi)
    {
        for (size_t qi = Nperim + 1; qi < points.size(); ++qi)
        {
            float d = std::hypot(points[qi].pos.x - points[pi].pos.x, points[qi].pos.y - points[pi].pos.y);
            if (d <= spacing * 1.5f)
            {
                springs.push_back({pi, (int)qi, d, stiffness_ring, false, 0});
            }
        }
    }

    // set perimeter count and compute and store target area (use polygon area from perimeter)
    N = Nperim;
    targetArea = compute_area(points, N);
    // set radius for heuristic
    radius = maxr;
}

// overload that takes BODY_PART tags per perimeter vertex and associates an owner id
void Jelly::create_filled_from_polygon(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts, float spacing, uint64_t id)
{
    if (pts.size() < 3)
    {
        // fallback: convert to simple vector and call base
        std::vector<sf::Vector2f> simple;
        for (auto &p : pts)
            simple.push_back(p.first);
        create_filled_from_polygon(simple, spacing);
        // tag nothing else
        for (auto &pt : points)
        {
            pt.body_part = BODY_PART::NONE;
            pt.id = id;
        }
        return;
    }

    // convert to plain vector of positions and call the primary factory
    std::vector<sf::Vector2f> simple;
    simple.reserve(pts.size());
    for (auto &p : pts)
        simple.push_back(p.first);

    create_filled_from_polygon(simple, spacing);

    // Now assign BODY_PART to the perimeter points (1..N) by matching coordinates
    // and set the owner id for all points.
    // Build a map from perimeter coordinate to BODY_PART using small distance matching.
    int perimCount = std::min((size_t)N, pts.size());
    for (int i = 1; i <= perimCount; ++i)
    {
        points[i].body_part = BODY_PART::NONE;
        points[i].id = id;
        // find closest input perimeter entry
        float bestd = 1e9f;
        BODY_PART bestPart = BODY_PART::NONE;
        for (auto &pp : pts)
        {
            float d = std::hypot(points[i].pos.x - pp.first.x, points[i].pos.y - pp.first.y);
            if (d < bestd)
            {
                bestd = d;
                bestPart = pp.second;
            }
        }
        points[i].body_part = bestPart;
    }
    // tag interior points and center as NONE but set owner id
    for (size_t i = perimCount + 1; i < points.size(); ++i)
    {
        points[i].body_part = BODY_PART::NONE;
        points[i].id = id;
    }
    if (!points.empty())
        points[0].id = id;
}

void Jelly::update_verlet(float dt, sf::Vector2f acceleration)
{
    // Reasonable gravity for soft-body character (not earth physics)
    const sf::Vector2f gravity{0.f, 400.f}; // Reduced from 981 for game feel

    // Apply PD targets to targeted parts before integration
    for (auto &t : partTargets)
    {
        if (!t.active)
            continue;
        // find matching point index
        int idx = find_part_index(t.owner, t.part);
        if (idx <= 0 || idx >= (int)points.size())
            continue;
        Point &p = points[idx];
        if (p.locked)
            continue;
        // approximate velocity
        sf::Vector2f vel = (p.pos - p.prev_pos);
        sf::Vector2f err = t.target - p.pos;
        sf::Vector2f force = err * t.kp - vel * t.kd;
        // convert PD force to point acceleration (F = m*a)
        p.acc += force / p.mass;
    }

    // apply pressure force as acceleration only on peripheral points (1..N)
    if (pressure != 0.f && targetArea > 0.f && N >= 3)
    {
        float currentArea = compute_area(points, N);
        if (currentArea > 1e-6f)
        {
            float areaRatio = currentArea / targetArea;
            // pressureFactor: positive if current < target (pull outward)
            float pressureFactor = (1.f - areaRatio) * pressure;
            for (int i = 1; i <= N; ++i)
            {
                Point &p = points[i];
                if (p.locked)
                    continue;
                sf::Vector2f dir = p.pos - points[0].pos;
                float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len > 1e-6f)
                {
                    dir /= len;
                    // scale pressure modestly
                    p.acc += dir * (pressureFactor * 500.f);
                }
            }
        }
    }

    for (auto &p : points)
    {
        if (p.locked)
            continue;
        sf::Vector2f a = acceleration + gravity + p.acc;
        sf::Vector2f velocity = (p.pos - p.prev_pos) * damping;
        sf::Vector2f new_pos = p.pos + velocity + a * dt * dt;
        p.prev_pos = p.pos;
        p.pos = new_pos;
        p.acc = {0.f, 0.f};
    }

    // STABLE CONSTRAINT SOLVER: Position-based dynamics approach
    for (int it = 0; it < iterations; ++it)
    {
        for (auto &s : springs)
        {
            Point &p1 = points[s.p1];
            Point &p2 = points[s.p2];
            if (p1.locked && p2.locked)
                continue;

            sf::Vector2f delta = p2.pos - p1.pos;
            float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            if (dist < 1e-3f) // Prevent division by zero
                continue;

            // Calculate constraint violation
            float error = (dist - s.rest_length) / dist;

            // Determine correction strength
            float correctionFactor = s.stiffness * stiffness;

            // Stronger correction for skeleton springs
            if (s.is_skeleton)
            {
                correctionFactor *= 1.5f;
            }

            // Limit correction to prevent instability
            correctionFactor = std::min(correctionFactor, 1.0f);

            // Calculate position correction
            sf::Vector2f correction = delta * error * correctionFactor;

            // Apply corrections based on mass ratio (heavier points move less)
            float mass1 = std::max(p1.mass, 0.1f); // Prevent tiny masses
            float mass2 = std::max(p2.mass, 0.1f);
            float totalMass = mass1 + mass2;

            float ratio1 = mass2 / totalMass; // Point 1 gets mass2's influence
            float ratio2 = mass1 / totalMass; // Point 2 gets mass1's influence

            if (!p1.locked)
            {
                p1.pos += correction * ratio1;
            }
            if (!p2.locked)
            {
                p2.pos -= correction * ratio2;
            }
        }
    }
}

void Jelly::update(float dt)
{
    update_verlet(dt);
}

// global force
void Jelly::apply_force(const sf::Vector2f &F_total)
{
    float M = 0.f;
    for (auto &p : points)
        M += p.mass;
    if (M == 0)
        return;

    sf::Vector2f a = F_total / M;
    for (auto &p : points)
        p.acc += a;
}

static float falloff(float d, float R)
{
    if (d >= R)
        return 0.f;
    float x = 1.f - d / R;
    return x * x;
}

void Jelly::apply_force(const sf::Vector2f &F_total, const sf::Vector2f &at, float radius)
{
    std::vector<float> w(points.size(), 0.f);
    float sumw = 0.f;

    for (size_t i = 0; i < points.size(); ++i)
    {
        float dx = points[i].pos.x - at.x;
        float dy = points[i].pos.y - at.y;
        float d = std::sqrt(dx * dx + dy * dy);
        w[i] = falloff(d, radius);
        sumw += w[i];
    }
    if (sumw <= 1e-6f)
        return;

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].locked || w[i] == 0.f)
            continue;
        sf::Vector2f Fi = (w[i] / sumw) * F_total;
        points[i].acc += Fi / points[i].mass;
    }
}

void Jelly::apply_force(const sf::Vector2f &J_total, const sf::Vector2f &at, float radius, float dt)
{
    std::vector<float> w(points.size(), 0.f);
    float sumw = 0.f;
    for (size_t i = 0; i < points.size(); ++i)
    {
        float d = std::hypot(points[i].pos.x - at.x, points[i].pos.y - at.y);
        w[i] = falloff(d, radius);
        sumw += w[i];
    }
    if (sumw <= 1e-6f)
        return;

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].locked || w[i] == 0.f)
            continue;
        sf::Vector2f Ji = (w[i] / sumw) * J_total;
        sf::Vector2f dv = Ji / points[i].mass;
        points[i].prev_pos -= dv * dt;
    }
}

void Jelly::draw(sf::RenderWindow &window)
{
    if (points.size() <= 1)
        return;

    // draw filled shape from peripheral points (1..N)
    int peripheralCount = N;
    if (peripheralCount >= 3 && (int)points.size() > peripheralCount)
    {
        sf::ConvexShape shape;
        shape.setPointCount(peripheralCount);
        for (int i = 0; i < peripheralCount; ++i)
        {
            shape.setPoint(i, points[i + 1].pos);
        }
        shape.setFillColor(sf::Color(120, 180, 240, 200));
        window.draw(shape);
    }

    // debug: draw point masses
    for (auto &p : points)
    {
        sf::CircleShape c(3.f);
        c.setOrigin(3.f, 3.f);
        c.setPosition(p.pos);
        c.setFillColor(sf::Color::Red);
        window.draw(c);
    }
}

void Jelly::create_from_points(const std::vector<std::pair<sf::Vector2f, BODY_PART>> &pts)
{
    if (pts.empty())
        return;
    // compute centroid
    sf::Vector2f c{0.f, 0.f};
    for (auto &p : pts)
        c += p.first;
    c /= float(pts.size());

    N = (int)pts.size();
    radius = 0.f;
    center = c;
    clear();

    points.resize(N + 1);
    points[0].pos = center;
    points[0].prev_pos = center;
    points[0].mass = 1.f;
    points[0].locked = false;

    for (int i = 1; i <= N; ++i)
    {
        points[i].pos = pts[i - 1].first;
        points[i].prev_pos = pts[i - 1].first;
        points[i].mass = 1.f;
        points[i].locked = false;
        points[i].body_part = pts[i - 1].second;
        float d = std::sqrt((points[i].pos.x - center.x) * (points[i].pos.x - center.x) + (points[i].pos.y - center.y) * (points[i].pos.y - center.y));
        radius = std::max(radius, d);
    }

    // springs
    springs.clear();
    for (int i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1;
        sf::Vector2f delta = points[next].pos - points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({i, next, dist, stiffness_ring, false, 0});

        sf::Vector2f delta2 = points[i].pos - points[0].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({0, i, dist2, stiffness_spoke, false, 0});
    }
    // compute and store target area for pressure (perimeter only)
    targetArea = compute_area(points, N);
}

// Add skeleton springs connecting canonical BODY_PART joints for the specified owner id.
void Jelly::add_skeleton_for_player(uint64_t id, float skeletonStiffness)
{
    if (points.empty())
        return;

    // Define canonical skeleton joints (only these will be connected with skeleton springs)
    std::vector<BODY_PART> canonicalJoints = {
        BODY_PART::HEAD, BODY_PART::NECK, BODY_PART::SPINE_UP, BODY_PART::SPINE_MID, BODY_PART::SPINE_LOW,
        BODY_PART::CLAV_R, BODY_PART::CLAV_L, BODY_PART::SHO_R, BODY_PART::SHO_L,
        BODY_PART::ELB_R, BODY_PART::ELB_L, BODY_PART::WRIST_R, BODY_PART::WRIST_L,
        BODY_PART::HAND_R, BODY_PART::HAND_L, BODY_PART::PELVIS,
        BODY_PART::HIP_R, BODY_PART::HIP_L, BODY_PART::KNEE_R, BODY_PART::KNEE_L,
        BODY_PART::ANKLE_R, BODY_PART::ANKLE_L, BODY_PART::FOOT_R, BODY_PART::FOOT_L};

    // map canonical BODY_PART -> point index for this owner id
    std::unordered_map<BODY_PART, int> part_index;

    // Search all points (not just perimeter) for canonical joints
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].id != id)
            continue;
        BODY_PART bp = points[i].body_part;
        if (bp == BODY_PART::NONE)
            continue;

        // Only map canonical skeleton joints
        bool isCanonical = false;
        for (BODY_PART canonical : canonicalJoints)
        {
            if (bp == canonical)
            {
                isCanonical = true;
                break;
            }
        }

        if (isCanonical && part_index.find(bp) == part_index.end())
        {
            part_index[bp] = (int)i;
        }
    }

    // helper to add a skeleton spring if both parts exist
    auto add_edge = [&](BODY_PART a, BODY_PART b)
    {
        if (part_index.find(a) == part_index.end() || part_index.find(b) == part_index.end())
            return;
        int ia = part_index[a];
        int ib = part_index[b];
        if (ia == ib)
            return;
        sf::Vector2f d = points[ib].pos - points[ia].pos;
        float dist = std::sqrt(d.x * d.x + d.y * d.y);
        // avoid zero-length
        if (dist < 1e-3f)
            dist = 1e-3f;
        springs.push_back({ia, ib, dist, skeletonStiffness, true, id});
    };

    // Canonical skeleton connections (proper game-standard rig)
    // Spine chain
    add_edge(BODY_PART::HEAD, BODY_PART::NECK);
    add_edge(BODY_PART::NECK, BODY_PART::SPINE_UP);
    add_edge(BODY_PART::SPINE_UP, BODY_PART::SPINE_MID);
    add_edge(BODY_PART::SPINE_MID, BODY_PART::SPINE_LOW);
    add_edge(BODY_PART::SPINE_LOW, BODY_PART::PELVIS);

    // Clavicles and shoulders
    add_edge(BODY_PART::NECK, BODY_PART::CLAV_R);
    add_edge(BODY_PART::NECK, BODY_PART::CLAV_L);
    add_edge(BODY_PART::CLAV_R, BODY_PART::SHO_R);
    add_edge(BODY_PART::CLAV_L, BODY_PART::SHO_L);

    // Arms
    add_edge(BODY_PART::SHO_R, BODY_PART::ELB_R);
    add_edge(BODY_PART::ELB_R, BODY_PART::WRIST_R);
    add_edge(BODY_PART::WRIST_R, BODY_PART::HAND_R);
    add_edge(BODY_PART::SHO_L, BODY_PART::ELB_L);
    add_edge(BODY_PART::ELB_L, BODY_PART::WRIST_L);
    add_edge(BODY_PART::WRIST_L, BODY_PART::HAND_L);

    // Legs
    add_edge(BODY_PART::PELVIS, BODY_PART::HIP_R);
    add_edge(BODY_PART::PELVIS, BODY_PART::HIP_L);
    add_edge(BODY_PART::HIP_R, BODY_PART::KNEE_R);
    add_edge(BODY_PART::KNEE_R, BODY_PART::ANKLE_R);
    add_edge(BODY_PART::ANKLE_R, BODY_PART::FOOT_R);
    add_edge(BODY_PART::HIP_L, BODY_PART::KNEE_L);
    add_edge(BODY_PART::KNEE_L, BODY_PART::ANKLE_L);
    add_edge(BODY_PART::ANKLE_L, BODY_PART::FOOT_L);

    // after adding skeleton springs, connect flesh to skeleton
    connect_flesh_to_skeleton(id, 0.35f); // softer stiffness for flesh connections
}

void Jelly::connect_flesh_to_skeleton(uint64_t owner_id, float fleshToSkeletonStiffness, int k, float maxDistance)
{
    if (points.empty())
        return;

    // build vector of skeleton joint indices (perimeter points with BODY_PART tags)
    std::vector<int> skeleton_indices;
    for (int i = 1; i <= N && i < (int)points.size(); ++i)
    {
        if (points[i].id == owner_id && points[i].body_part != BODY_PART::NONE)
        {
            skeleton_indices.push_back(i);
        }
    }

    if (skeleton_indices.empty())
        return; // no skeleton joints found

    // for each interior/flesh point, connect to k nearest skeleton joints
    for (size_t i = 0; i < points.size(); ++i)
    {
        // skip if not owned by this player or if it's already a skeleton joint
        if (points[i].id != owner_id)
            continue;
        if (i > 0 && i <= (size_t)N && points[i].body_part != BODY_PART::NONE)
            continue; // skip perimeter skeleton points

        // find k nearest skeleton joints within maxDistance
        std::vector<std::pair<float, int>> distances;
        for (int skel_idx : skeleton_indices)
        {
            float dx = points[skel_idx].pos.x - points[i].pos.x;
            float dy = points[skel_idx].pos.y - points[i].pos.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= maxDistance)
            {
                distances.push_back({dist, skel_idx});
            }
        }

        // sort by distance and take k nearest
        std::sort(distances.begin(), distances.end());
        int connections = std::min(k, (int)distances.size());

        for (int j = 0; j < connections; ++j)
        {
            float dist = distances[j].first;
            int skel_idx = distances[j].second;

            // ensure minimum rest length
            if (dist < 1e-3f)
                dist = 1e-3f;

            // check for duplicate spring
            bool duplicate = false;
            for (const auto &s : springs)
            {
                if ((s.p1 == (int)i && s.p2 == skel_idx) || (s.p1 == skel_idx && s.p2 == (int)i))
                {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate)
            {
                springs.push_back({(int)i, skel_idx, dist, fleshToSkeletonStiffness, false, owner_id});
            }
        }
    }
}

// Apply postural stability forces to help character stand upright
void Jelly::apply_postural_stability(uint64_t owner_id, float dt, float ground_y)
{
    if (points.empty())
        return;

    // Find key body parts for postural control
    std::unordered_map<BODY_PART, int> part_indices;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].id == owner_id && points[i].body_part != BODY_PART::NONE)
        {
            part_indices[points[i].body_part] = (int)i;
        }
    }

    // === CORE POSTURAL CONTROLLER (like Euphoria/GTA ragdoll physics) ===

    // 1. FOOT PLANTING: Lock feet when grounded (critical for stability)
    auto plantFoot = [&](BODY_PART footPart)
    {
        auto it = part_indices.find(footPart);
        if (it == part_indices.end())
            return;

        Point &foot = points[it->second];
        float distToGround = ground_y - foot.pos.y;

        if (distToGround < 15.f && distToGround > -2.f) // Grounded
        {
            // STRONG foot planting - this is crucial for standing
            foot.mass = 12.0f; // Very heavy when grounded

            // Active downward force to "stick" to ground
            foot.acc.y += 300.f;

            // Reduce horizontal drift significantly (like real foot plant)
            sf::Vector2f vel = foot.pos - foot.prev_pos;
            if (std::abs(vel.x) < 30.f) // Static friction range
            {
                foot.prev_pos.x = foot.pos.x - vel.x * 0.1f; // Almost lock horizontal
            }
        }
        else
        {
            foot.mass = std::max(foot.mass * 0.9f, 2.0f); // Reduce when airborne
        }
    };

    plantFoot(BODY_PART::FOOT_L);
    plantFoot(BODY_PART::FOOT_R);

    // 2. SPINE RIGIDITY: Keep spine straight and vertical (like real skeleton)
    auto rigidSpine = [&]()
    {
        auto pelvis_it = part_indices.find(BODY_PART::PELVIS);
        auto spine_low_it = part_indices.find(BODY_PART::SPINE_LOW);
        auto spine_mid_it = part_indices.find(BODY_PART::SPINE_MID);
        auto spine_up_it = part_indices.find(BODY_PART::SPINE_UP);
        auto head_it = part_indices.find(BODY_PART::HEAD);

        // Apply strong vertical alignment to entire spine chain
        std::vector<int> spineChain;
        if (pelvis_it != part_indices.end())
            spineChain.push_back(pelvis_it->second);
        if (spine_low_it != part_indices.end())
            spineChain.push_back(spine_low_it->second);
        if (spine_mid_it != part_indices.end())
            spineChain.push_back(spine_mid_it->second);
        if (spine_up_it != part_indices.end())
            spineChain.push_back(spine_up_it->second);
        if (head_it != part_indices.end())
            spineChain.push_back(head_it->second);

        if (spineChain.size() >= 2)
        {
            // Force spine segments to maintain vertical alignment
            for (size_t i = 1; i < spineChain.size(); ++i)
            {
                Point &lower = points[spineChain[i - 1]];
                Point &upper = points[spineChain[i]];

                // Calculate target position (upper should be directly above lower)
                sf::Vector2f idealOffset = {0.f, -40.f}; // 40 pixels up
                sf::Vector2f currentOffset = upper.pos - lower.pos;
                sf::Vector2f correction = idealOffset - currentOffset;

                // Apply strong corrective forces
                float strength = 250.f;
                if (!upper.locked)
                    upper.acc += correction * strength * dt / upper.mass;
                if (!lower.locked)
                    lower.acc -= correction * strength * dt * 0.5f / lower.mass;
            }
        }
    };
    rigidSpine();

    // 3. ACTIVE BALANCE CONTROLLER: Keep center of mass over base of support
    auto activeBalance = [&]()
    {
        auto foot_l_it = part_indices.find(BODY_PART::FOOT_L);
        auto foot_r_it = part_indices.find(BODY_PART::FOOT_R);
        auto pelvis_it = part_indices.find(BODY_PART::PELVIS);

        if (foot_l_it == part_indices.end() || foot_r_it == part_indices.end() ||
            pelvis_it == part_indices.end())
            return;

        Point &footL = points[foot_l_it->second];
        Point &footR = points[foot_r_it->second];
        Point &pelvis = points[pelvis_it->second];

        // Check if both feet are grounded
        bool leftGrounded = (ground_y - footL.pos.y) < 15.f;
        bool rightGrounded = (ground_y - footR.pos.y) < 15.f;

        if (leftGrounded && rightGrounded)
        {
            // Calculate base of support (center between feet)
            sf::Vector2f supportCenter = (footL.pos + footR.pos) * 0.5f;

            // Calculate total center of mass
            sf::Vector2f com = {0.f, 0.f};
            float totalMass = 0.f;
            for (const auto &p : points)
            {
                if (p.id == owner_id)
                {
                    com += p.pos * p.mass;
                    totalMass += p.mass;
                }
            }
            if (totalMass > 1e-6f)
                com /= totalMass;

            // Apply STRONG balance control to keep COM over base
            float horizontalError = com.x - supportCenter.x;

            if (std::abs(horizontalError) > 3.f) // Very sensitive balance
            {
                // Apply counter-torque through pelvis (like real human balance)
                sf::Vector2f balanceForce = {-horizontalError * 400.f, 0.f};
                pelvis.acc += balanceForce / pelvis.mass;

                // Also bias upper body to counter-balance
                auto applyCounterBalance = [&](BODY_PART part, float weight)
                {
                    auto it = part_indices.find(part);
                    if (it != part_indices.end() && !points[it->second].locked)
                    {
                        points[it->second].acc += balanceForce * weight / points[it->second].mass;
                    }
                };

                applyCounterBalance(BODY_PART::SPINE_UP, 0.6f);
                applyCounterBalance(BODY_PART::HEAD, 0.4f);
            }
        }
    };
    activeBalance();

    // 4. HIP HEIGHT MAINTENANCE: Keep pelvis at proper height above feet
    auto maintainHipHeight = [&]()
    {
        auto pelvis_it = part_indices.find(BODY_PART::PELVIS);
        auto foot_l_it = part_indices.find(BODY_PART::FOOT_L);
        auto foot_r_it = part_indices.find(BODY_PART::FOOT_R);

        if (pelvis_it == part_indices.end() || foot_l_it == part_indices.end() ||
            foot_r_it == part_indices.end())
            return;

        Point &pelvis = points[pelvis_it->second];
        Point &footL = points[foot_l_it->second];
        Point &footR = points[foot_r_it->second];

        // Target pelvis height above feet
        float avgFootY = (footL.pos.y + footR.pos.y) * 0.5f;
        float targetPelvisY = avgFootY - 120.f; // Pelvis should be 120px above feet
        float currentPelvisY = pelvis.pos.y;

        float heightError = currentPelvisY - targetPelvisY;

        if (std::abs(heightError) > 10.f)
        {
            // Apply vertical correction force
            sf::Vector2f heightForce = {0.f, -heightError * 200.f};
            pelvis.acc += heightForce / pelvis.mass;
        }
    };
    maintainHipHeight();
}
