#include "jelly.hpp"
#include <cmath>
#include <unordered_map>
#include <deque>

// Helper function for unique ID generation
uint64_t generateID()
{
    static uint64_t counter = 1;
    return counter++;
}

void Jelly::add_point(const sf::Vector2f &pos, BODY_PART body_part)
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

void Jelly::add_edge(BODY_PART a, BODY_PART b)
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

// Default constructor delegates to param constructor
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
            s.rest_length += (newRest - s.rest_length) * lerp;
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
            sf::Vector2f interpolated = pts[i].first + t * (pts[j].first - pts[i].first);
            res.push_back(interpolated);
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
            sf::Vector2f candidate{x, y};
            if (point_in_polygon(candidate, pts))
            {
                Point newPt;
                newPt.pos = candidate;
                newPt.prev_pos = candidate;
                newPt.mass = 1.f;
                newPt.locked = false;
                newPt.body_part = BODY_PART::NONE;
                newPt.id = 0;
                points.push_back(newPt);
                grid[r][cidx] = (int)points.size() - 1;
            }
        }
    }

    // create lattice springs connecting neighbors in triangular grid
    for (int r = 0; r < rows; ++r)
    {
        for (int cidx = 0; cidx < cols; ++cidx)
        {
            int current = grid[r][cidx];
            if (current == -1)
                continue;

            // connect to right neighbor
            if (cidx + 1 < cols && grid[r][cidx + 1] != -1)
            {
                int neighbor = grid[r][cidx + 1];
                sf::Vector2f delta = points[neighbor].pos - points[current].pos;
                float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
                springs.push_back({current, neighbor, dist, stiffness_flesh, false, 0});
            }

            // connect to lower neighbors (triangular pattern)
            if (r + 1 < rows)
            {
                int offset = (r % 2) ? -1 : 0;
                for (int dc = offset; dc <= offset + 1; ++dc)
                {
                    int nc = cidx + dc;
                    if (nc >= 0 && nc < cols && grid[r + 1][nc] != -1)
                    {
                        int neighbor = grid[r + 1][nc];
                        sf::Vector2f delta = points[neighbor].pos - points[current].pos;
                        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
                        springs.push_back({current, neighbor, dist, stiffness_flesh, false, 0});
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
            sf::Vector2f delta = points[qi].pos - points[pi].pos;
            float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            if (dist < spacing * 1.5f)
            {
                springs.push_back({pi, (int)qi, dist, stiffness_flesh, false, 0});
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
            pt.id = id;
            pt.body_part = BODY_PART::NONE;
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
            sf::Vector2f diff = pp.first - points[i].pos;
            float d = std::sqrt(diff.x * diff.x + diff.y * diff.y);
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

    // 🎮 GANG BEASTS EVOLUTION: Apply mass multipliers to points
    // RE-ENABLED FOR TESTING
    if (gangBeastsSettings)
    {
        for (auto &p : points)
        {
            if (p.body_part != BODY_PART::NONE)
            {
                float massMultiplier = getMassMultiplier(p.body_part);
                // Apply mass multiplier (but preserve original base mass)
                if (p.mass < 0.1f)
                { // Only modify if it's still base mass
                    p.mass = 0.0001f * massMultiplier;
                }
            }
        }
    }

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
            float areaError = targetArea - currentArea;
            float pressureForce = areaError * pressure;
            // distribute radially from center
            sf::Vector2f centroid{0.f, 0.f};
            for (int i = 1; i <= N; ++i)
                centroid += points[i].pos;
            centroid /= float(N);
            for (int i = 1; i <= N; ++i)
            {
                sf::Vector2f radial = points[i].pos - centroid;
                float len = std::sqrt(radial.x * radial.x + radial.y * radial.y);
                if (len > 1e-6f)
                {
                    radial /= len;
                    points[i].acc += radial * pressureForce / points[i].mass;
                }
            }
        }
    }

    // 🎮 GANG BEASTS EVOLUTION: Apply enhanced flesh damping
    // RE-ENABLED FOR TESTING
    float fleshDampingMultiplier = gangBeastsSettings ? getFleshDampingMultiplier() : 1.0f;

    for (auto &p : points)
    {
        if (p.locked)
            continue;
        sf::Vector2f a = acceleration + gravity + p.acc;

        // Apply Gang Beasts flesh damping for more wobbliness
        float effectiveDamping = damping;
        if (p.body_part == BODY_PART::NONE)
        { // Flesh points
            effectiveDamping *= fleshDampingMultiplier;
        }

        sf::Vector2f velocity = (p.pos - p.prev_pos) * effectiveDamping;
        sf::Vector2f new_pos = p.pos + velocity + a * dt * dt;
        p.prev_pos = p.pos;
        p.pos = new_pos;
        p.acc = {0.f, 0.f};
    }

    // 🎮 GANG BEASTS EVOLUTION: Update spring fatigue and advanced systems
    // RE-ENABLED FOR TESTING
    if (gangBeastsSettings)
    {
        updateSpringFatigue(dt);
        updateAdvancedSpringSystems(dt); // Step 1.3: Advanced Spring System
    }

    // STABLE CONSTRAINT SOLVER: Position-based dynamics approach
    for (int it = 0; it < iterations; ++it)
    {
        for (size_t springIdx = 0; springIdx < springs.size(); ++springIdx)
        {
            Spring &s = springs[springIdx];
            Point &p1 = points[s.p1];
            Point &p2 = points[s.p2];
            if (p1.locked && p2.locked)
                continue;

            sf::Vector2f delta = p2.pos - p1.pos;
            float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            if (dist < 1e-6f)
                continue;

            float error = dist - s.rest_length;
            sf::Vector2f correction = (delta / dist) * error * 0.5f;

            // Apply hierarchy-based stiffness
            float effectiveStiffness = stiffness * s.stiffness;
            if (s.is_skeleton)
            {
                effectiveStiffness *= stiffness_bone; // Skeleton springs are very rigid

                // 🎮 GANG BEASTS EVOLUTION: Apply skeleton stiffness multiplier
                // RE-ENABLED FOR TESTING
                if (gangBeastsSettings)
                {
                    float gangBeastsMultiplier = getSkeletonStiffnessMultiplier(p1.body_part, p2.body_part);
                    effectiveStiffness *= gangBeastsMultiplier;
                }

                // 🎮 GANG BEASTS EVOLUTION: Apply spring fatigue and advanced systems
                // RE-ENABLED FOR TESTING ADVANCED SPRING SYSTEM
                if (gangBeastsSettings)
                {
                    applySpringFatigue(springIdx, effectiveStiffness);

                    // Step 1.3: Apply advanced spring system effects
                    if (s.owner_id != 0)
                    {
                        // Apply context-aware stiffness
                        auto springStateIt = advancedSpringStates.find(s.owner_id);
                        if (springStateIt != advancedSpringStates.end())
                        {
                            effectiveStiffness *= springStateIt->second.current_stiffness_multiplier;

                            // Apply spring strength overrides
                            auto overrideIt = springStateIt->second.spring_strength_overrides.find(springIdx);
                            if (overrideIt != springStateIt->second.spring_strength_overrides.end())
                            {
                                effectiveStiffness *= overrideIt->second;
                            }
                        }

                        // Apply adaptive physics (but maintain original behavior by default)
                        applyAdaptivePhysics(s.owner_id, springIdx, effectiveStiffness);
                    }
                }
            }
            else
            {
                // Regular springs use ring/spoke stiffness
                effectiveStiffness *= (s.p1 == 0 || s.p2 == 0) ? stiffness_spoke : stiffness_ring;

                // 🎮 GANG BEASTS EVOLUTION: Apply flesh spring reduction
                // RE-ENABLED FOR TESTING
                if (gangBeastsSettings && p1.body_part == BODY_PART::NONE && p2.body_part == BODY_PART::NONE)
                {
                    effectiveStiffness *= (1.0f - gangBeastsSettings->physics_personality.flesh_springs.stiffness_reduction);
                }
            }

            correction *= effectiveStiffness;

            // Apply mass-weighted corrections
            float totalMass = p1.mass + p2.mass;
            if (totalMass > 1e-6f)
            {
                float mass1_ratio = p2.mass / totalMass;
                float mass2_ratio = p1.mass / totalMass;

                if (!p1.locked)
                    p1.pos += correction * mass1_ratio;
                if (!p2.locked)
                    p2.pos -= correction * mass2_ratio;
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
        if (points[i].id == id && points[i].body_part != BODY_PART::NONE)
        {
            // Check if this is a canonical joint
            for (BODY_PART canonical : canonicalJoints)
            {
                if (points[i].body_part == canonical)
                {
                    part_index[canonical] = (int)i;
                    break;
                }
            }
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
        if (dist < 1e-3f)
            return;
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
    connect_flesh_to_skeleton(id, 0.35f, 1, 35.0f); // CRITICAL: Only 1 connection per flesh point, short range for independence
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
        if (points[i].id != owner_id || points[i].body_part != BODY_PART::NONE)
            continue; // skip non-flesh or wrong owner

        // find k nearest skeleton joints
        std::vector<std::pair<float, int>> distances;
        for (int skel_idx : skeleton_indices)
        {
            sf::Vector2f diff = points[skel_idx].pos - points[i].pos;
            float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
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
            int skel_idx = distances[j].second;
            float dist = distances[j].first;
            springs.push_back({(int)i, skel_idx, dist, fleshToSkeletonStiffness, false, owner_id});
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
        if (part_indices.find(footPart) == part_indices.end())
            return;
        int idx = part_indices[footPart];
        Point &foot = points[idx];

        // Check if foot is near ground
        if (foot.pos.y >= ground_y - 5.0f)
        {
            foot.pos.y = ground_y;      // Snap to ground
            foot.prev_pos.y = ground_y; // Stop vertical movement
            // Add some horizontal friction
            foot.prev_pos.x = foot.pos.x + (foot.prev_pos.x - foot.pos.x) * 0.8f;
        }
    };

    plantFoot(BODY_PART::FOOT_L);
    plantFoot(BODY_PART::FOOT_R);

    // 2. SPINE RIGIDITY: Keep spine straight and vertical (like real skeleton)
    auto rigidSpine = [&]()
    {
        std::vector<BODY_PART> spineChain = {
            BODY_PART::PELVIS, BODY_PART::SPINE_LOW, BODY_PART::SPINE_MID,
            BODY_PART::SPINE_UP, BODY_PART::NECK, BODY_PART::HEAD};

        for (size_t i = 0; i < spineChain.size() - 1; ++i)
        {
            if (part_indices.find(spineChain[i]) == part_indices.end() ||
                part_indices.find(spineChain[i + 1]) == part_indices.end())
                continue;

            int lower_idx = part_indices[spineChain[i]];
            int upper_idx = part_indices[spineChain[i + 1]];

            Point &lower = points[lower_idx];
            Point &upper = points[upper_idx];

            // Apply upward bias to maintain vertical alignment
            sf::Vector2f spineDir = upper.pos - lower.pos;
            float len = std::sqrt(spineDir.x * spineDir.x + spineDir.y * spineDir.y);
            if (len > 1e-6f)
            {
                spineDir /= len;
                sf::Vector2f uprightDir{0.0f, -1.0f}; // Up direction
                sf::Vector2f correctionForce = (uprightDir - spineDir) * 50.0f;

                upper.acc += correctionForce / upper.mass;
                lower.acc -= correctionForce / lower.mass;
            }
        }
    };
    rigidSpine();

    // 3. ACTIVE BALANCE CONTROLLER: Keep center of mass over base of support
    auto activeBalance = [&]()
    {
        // Find center of mass
        sf::Vector2f com{0.0f, 0.0f};
        float total_mass = 0.0f;
        for (auto &p : points)
        {
            if (p.id == owner_id)
            {
                com += p.pos * p.mass;
                total_mass += p.mass;
            }
        }
        if (total_mass > 1e-6f)
            com /= total_mass;

        // Find base of support (average foot position)
        sf::Vector2f base_center{0.0f, 0.0f};
        int foot_count = 0;
        if (part_indices.find(BODY_PART::FOOT_L) != part_indices.end())
        {
            base_center += points[part_indices[BODY_PART::FOOT_L]].pos;
            foot_count++;
        }
        if (part_indices.find(BODY_PART::FOOT_R) != part_indices.end())
        {
            base_center += points[part_indices[BODY_PART::FOOT_R]].pos;
            foot_count++;
        }
        if (foot_count > 0)
        {
            base_center /= float(foot_count);

            // Apply corrective force to pelvis to shift COM over base
            if (part_indices.find(BODY_PART::PELVIS) != part_indices.end())
            {
                int pelvis_idx = part_indices[BODY_PART::PELVIS];
                Point &pelvis = points[pelvis_idx];

                sf::Vector2f com_error = base_center - com;
                com_error.y = 0.0f; // Only horizontal correction
                sf::Vector2f balance_force = com_error * 30.0f;

                pelvis.acc += balance_force / pelvis.mass;
            }
        }
    };
    activeBalance();

    // 4. HIP HEIGHT MAINTENANCE: Keep pelvis at proper height above feet
    auto maintainHipHeight = [&]()
    {
        if (part_indices.find(BODY_PART::PELVIS) == part_indices.end())
            return;

        int pelvis_idx = part_indices[BODY_PART::PELVIS];
        Point &pelvis = points[pelvis_idx];

        // Target hip height (approximate human proportions)
        float target_hip_height = ground_y - 90.0f;

        if (pelvis.pos.y > target_hip_height)
        {
            float height_error = target_hip_height - pelvis.pos.y;
            sf::Vector2f lift_force{0.0f, height_error * 20.0f};
            pelvis.acc += lift_force / pelvis.mass;
        }
    };
    maintainHipHeight();
}

// 🎮 GANG BEASTS EVOLUTION: Settings Integration Implementation

void Jelly::setGangBeastsSettings(const GangBeastsSettings::GangBeastsPhysicsSettings *settings)
{
    gangBeastsSettings = settings;
    if (settings)
    {
        std::cout << "🎮 Gang Beasts physics settings applied to Jelly " << id << std::endl;
    }
}

float Jelly::getSkeletonStiffnessMultiplier(BODY_PART partA, BODY_PART partB) const
{
    if (!gangBeastsSettings)
        return 1.0f;

    const auto &skeleton = gangBeastsSettings->physics_personality.skeleton_springs;
    float baseMultiplier = skeleton.base_stiffness_multiplier;

    // Determine joint type and apply specific looseness factor
    auto isSpineJoint = [](BODY_PART part)
    {
        return part == BODY_PART::HEAD || part == BODY_PART::NECK ||
               part == BODY_PART::SPINE_UP || part == BODY_PART::SPINE_MID ||
               part == BODY_PART::SPINE_LOW || part == BODY_PART::PELVIS;
    };

    auto isArmJoint = [](BODY_PART part)
    {
        return part == BODY_PART::CLAV_R || part == BODY_PART::CLAV_L ||
               part == BODY_PART::SHO_R || part == BODY_PART::SHO_L ||
               part == BODY_PART::ELB_R || part == BODY_PART::ELB_L;
    };

    auto isWristJoint = [](BODY_PART part)
    {
        return part == BODY_PART::WRIST_R || part == BODY_PART::WRIST_L ||
               part == BODY_PART::HAND_R || part == BODY_PART::HAND_L;
    };

    auto isLegJoint = [](BODY_PART part)
    {
        return part == BODY_PART::HIP_R || part == BODY_PART::HIP_L ||
               part == BODY_PART::KNEE_R || part == BODY_PART::KNEE_L;
    };

    auto isAnkleJoint = [](BODY_PART part)
    {
        return part == BODY_PART::ANKLE_R || part == BODY_PART::ANKLE_L ||
               part == BODY_PART::FOOT_R || part == BODY_PART::FOOT_L;
    };

    // Apply joint-specific looseness
    float jointFactor = 1.0f;
    if (isSpineJoint(partA) || isSpineJoint(partB))
    {
        jointFactor = skeleton.spine_joints;
    }
    else if (isWristJoint(partA) || isWristJoint(partB))
    {
        jointFactor = skeleton.wrist_joints;
    }
    else if (isArmJoint(partA) || isArmJoint(partB))
    {
        jointFactor = skeleton.arm_joints;
    }
    else if (isAnkleJoint(partA) || isAnkleJoint(partB))
    {
        jointFactor = skeleton.ankle_joints;
    }
    else if (isLegJoint(partA) || isLegJoint(partB))
    {
        jointFactor = skeleton.leg_joints;
    }
    else if (partA == BODY_PART::NECK || partB == BODY_PART::NECK)
    {
        jointFactor = skeleton.neck_joint;
    }

    return baseMultiplier * jointFactor;
}

float Jelly::getFleshDampingMultiplier() const
{
    if (!gangBeastsSettings)
        return 1.0f;
    return gangBeastsSettings->physics_personality.flesh_springs.damping_multiplier;
}

float Jelly::getMassMultiplier(BODY_PART bodyPart) const
{
    if (!gangBeastsSettings)
        return 1.0f;

    const auto &mass = gangBeastsSettings->physics_personality.mass_distribution;

    switch (bodyPart)
    {
    case BODY_PART::HEAD:
        return mass.head_mass_multiplier;
    case BODY_PART::NECK:
    case BODY_PART::SPINE_UP:
    case BODY_PART::SPINE_MID:
    case BODY_PART::SPINE_LOW:
    case BODY_PART::PELVIS:
    case BODY_PART::CLAV_R:
    case BODY_PART::CLAV_L:
        return mass.torso_mass_multiplier;
    default:
        return mass.limb_mass_multiplier;
    }
}

void Jelly::updateSpringFatigue(float dt)
{
    if (!gangBeastsSettings || !gangBeastsSettings->physics_personality.spring_fatigue.enabled)
    {
        return;
    }

    const auto &fatigue = gangBeastsSettings->physics_personality.spring_fatigue;

    for (size_t i = 0; i < springs.size(); ++i)
    {
        const Spring &spring = springs[i];

        // Skip non-skeleton springs for fatigue
        if (!spring.is_skeleton)
            continue;

        auto &state = springFatigueStates[i];

        // Calculate current spring stress (how much it's stretched/compressed)
        sf::Vector2f delta = points[spring.p2].pos - points[spring.p1].pos;
        float current_length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        float stress = std::abs(current_length - spring.rest_length) / spring.rest_length;

        // Apply fatigue based on stress
        if (stress > 0.1f)
        { // 10% deformation threshold
            state.fatigue_accumulation += stress * fatigue.fatigue_rate * dt;
        }
        else
        {
            // Recovery when not stressed
            state.fatigue_accumulation -= fatigue.recovery_rate * dt;
        }

        // Clamp fatigue accumulation
        state.fatigue_accumulation = std::max(0.0f, std::min(1.0f, state.fatigue_accumulation));

        // Calculate current strength
        state.current_strength = fatigue.max_strength -
                                 (state.fatigue_accumulation * (fatigue.max_strength - fatigue.min_strength));

        state.is_fatigued = state.current_strength < 0.8f;
    }
}

void Jelly::applySpringFatigue(int springIndex, float &stiffness)
{
    if (!gangBeastsSettings || !gangBeastsSettings->physics_personality.spring_fatigue.enabled)
    {
        return;
    }

    auto it = springFatigueStates.find(springIndex);
    if (it != springFatigueStates.end())
    {
        stiffness *= it->second.current_strength;
    }
}

void Jelly::apply_enhanced_postural_stability(uint64_t owner_id, float dt, float ground_y)
{
    if (!gangBeastsSettings)
    {
        // Fallback to original stability system
        apply_postural_stability(owner_id, dt, ground_y);
        return;
    }

    // Step 1.2: Enhanced postural stability with Gang Beasts personality
    auto &posturalState = posturalStates[owner_id];
    const auto &stability = gangBeastsSettings->postural_stability;

    // Calculate current imbalance magnitude
    float imbalance = calculateImbalanceMagnitude(owner_id, ground_y);
    posturalState.imbalance_magnitude = imbalance;

    // Check if should enter panic mode
    bool shouldPanic = shouldEnterPanicMode(owner_id);
    if (shouldPanic && !posturalState.is_panicking)
    {
        posturalState.is_panicking = true;
        posturalState.reaction_timer = stability.reaction_timing.panic_reaction_delay;
        if (gangBeastsSettings->debug.log_physics_events)
        {
            std::cout << "🚨 Player " << owner_id << " entering panic mode!" << std::endl;
        }
    }
    else if (!shouldPanic && posturalState.is_panicking)
    {
        posturalState.is_panicking = false;
        posturalState.reaction_timer = stability.reaction_timing.base_reaction_delay;
    }

    // Apply reaction delay
    applyReactionDelay(owner_id, dt);

    // If reaction timer hasn't expired, don't apply stability forces yet
    if (posturalState.reaction_timer > 0.0f)
    {
        return;
    }

    // Apply original stability with modifications
    apply_postural_stability(owner_id, dt, ground_y);

    // Apply Gang Beasts enhancements after base stability
    if (stability.overcompensation.enabled)
    {
        // Get recent stability forces and apply overcompensation
        sf::Vector2f stabilityForce = posturalState.stability_momentum;
        applyOvercompensation(owner_id, stabilityForce);

        // Apply the overcompensated forces
        auto part_indices = std::unordered_map<BODY_PART, int>();
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (points[i].id == owner_id && points[i].body_part != BODY_PART::NONE)
            {
                part_indices[points[i].body_part] = (int)i;
            }
        }

        if (part_indices.find(BODY_PART::PELVIS) != part_indices.end())
        {
            int pelvis_idx = part_indices[BODY_PART::PELVIS];
            points[pelvis_idx].acc += stabilityForce / points[pelvis_idx].mass;
        }
    }

    // Update wobble decay
    if (posturalState.wobble_decay_timer > 0.0f)
    {
        posturalState.wobble_decay_timer -= dt;
        posturalState.stability_momentum *= stability.overcompensation.oscillation_damping;
    }
}

float Jelly::calculateImbalanceMagnitude(uint64_t owner_id, float ground_y)
{
    // Find center of mass and base of support
    sf::Vector2f com{0.0f, 0.0f};
    float total_mass = 0.0f;
    sf::Vector2f base_center{0.0f, 0.0f};
    int foot_count = 0;

    for (auto &p : points)
    {
        if (p.id == owner_id)
        {
            com += p.pos * p.mass;
            total_mass += p.mass;

            if (p.body_part == BODY_PART::FOOT_L || p.body_part == BODY_PART::FOOT_R)
            {
                base_center += p.pos;
                foot_count++;
            }
        }
    }

    if (total_mass > 1e-6f)
        com /= total_mass;
    if (foot_count > 0)
        base_center /= float(foot_count);

    // Calculate horizontal displacement of COM from base
    sf::Vector2f displacement = com - base_center;
    displacement.y = 0.0f; // Only horizontal matters for balance

    return std::sqrt(displacement.x * displacement.x + displacement.y * displacement.y);
}

void Jelly::applyReactionDelay(uint64_t owner_id, float dt)
{
    auto &posturalState = posturalStates[owner_id];
    const auto &timing = gangBeastsSettings->postural_stability.reaction_timing;

    if (posturalState.reaction_timer > 0.0f)
    {
        posturalState.reaction_timer -= dt;

        // Speed up reaction if imbalance is getting worse
        if (posturalState.imbalance_magnitude > 0.2f)
        {
            posturalState.reaction_timer -= dt * (timing.recovery_acceleration - 1.0f);
        }

        posturalState.reaction_timer = std::max(0.0f, posturalState.reaction_timer);
    }
}

void Jelly::applyOvercompensation(uint64_t owner_id, sf::Vector2f &stabilityForce)
{
    auto &posturalState = posturalStates[owner_id];
    const auto &overcomp = gangBeastsSettings->postural_stability.overcompensation;

    // Amplify the stability force
    stabilityForce *= overcomp.overcompensation_factor;

    // Add to momentum for oscillation effects
    posturalState.stability_momentum += stabilityForce * 0.1f;

    // Start wobble decay timer
    posturalState.wobble_decay_timer = overcomp.wobble_decay_time;

    // Add some randomness for comedy
    if (posturalState.is_panicking)
    {
        sf::Vector2f randomWobble{
            (std::rand() % 200 - 100) / 100.0f * 5.0f,
            0.0f};
        stabilityForce += randomWobble;
    }
}

bool Jelly::shouldEnterPanicMode(uint64_t owner_id)
{
    const auto &sensitivity = gangBeastsSettings->postural_stability.sensitivity;
    auto &posturalState = posturalStates[owner_id];

    return posturalState.imbalance_magnitude > sensitivity.panic_threshold;
}

// ====================================================================
// 🔧 STEP 1.3: ADVANCED SPRING SYSTEM IMPLEMENTATION
// ====================================================================

void Jelly::updateAdvancedSpringSystems(float dt)
{
    if (!hasGangBeastsSettings())
        return;

    // Update all advanced spring system components for each owner
    for (auto &[owner_id, springState] : advancedSpringStates)
    {
        updateContextAwareStiffness(owner_id, dt);
        updateSpringAnimationSystem(owner_id, dt);
        updateSpringStrengthModulation(owner_id, dt);
        updateAdaptivePhysics(owner_id, dt);
    }
}

void Jelly::updateContextAwareStiffness(uint64_t owner_id, float dt)
{
    if (!gangBeastsSettings->advanced_spring_system.context_aware_stiffness.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];
    const auto &contextStiffness = gangBeastsSettings->advanced_spring_system.context_aware_stiffness;

    // Detect current physics state
    PhysicsState newState = detectPhysicsState(owner_id);

    // Handle state transitions
    if (newState != springState.current_state)
    {
        springState.previous_state = springState.current_state;
        springState.current_state = newState;
        springState.transition_timer = 0.0f;
        springState.state_duration = 0.0f;
    }

    // Update timers
    springState.transition_timer += dt;
    springState.state_duration += dt;

    // Determine target stiffness based on state
    float targetMultiplier = 1.0f; // Default (original behavior)
    switch (springState.current_state)
    {
    case PhysicsState::WALKING:
        targetMultiplier = contextStiffness.walking_stiffness_multiplier;
        break;
    case PhysicsState::JUMPING:
        targetMultiplier = contextStiffness.jumping_stiffness_multiplier;
        break;
    case PhysicsState::FALLING:
        targetMultiplier = contextStiffness.falling_stiffness_multiplier;
        break;
    case PhysicsState::GRABBING:
        targetMultiplier = contextStiffness.grabbing_stiffness_multiplier;
        break;
    case PhysicsState::IDLE:
    default:
        targetMultiplier = contextStiffness.idle_stiffness_multiplier;
        break;
    }

    springState.target_stiffness_multiplier = targetMultiplier;

    // Smooth transition to target stiffness
    float transitionSpeed = contextStiffness.transition_speed;
    float diff = springState.target_stiffness_multiplier - springState.current_stiffness_multiplier;
    springState.current_stiffness_multiplier += diff * transitionSpeed * dt;

    // Clamp to prevent instability
    springState.current_stiffness_multiplier = std::max(0.1f, std::min(2.0f, springState.current_stiffness_multiplier));
}

void Jelly::updateSpringAnimationSystem(uint64_t owner_id, float dt)
{
    if (!gangBeastsSettings->advanced_spring_system.spring_animation_system.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];
    const auto &animSystem = gangBeastsSettings->advanced_spring_system.spring_animation_system;

    // Update active animations
    for (auto &animation : springState.active_animations)
    {
        animation.progress += dt / animation.duration;

        // Handle looping animations
        if (animation.is_looping && animation.progress >= 1.0f)
        {
            animation.progress = std::fmod(animation.progress, 1.0f);
        }
    }

    // Remove completed non-looping animations
    springState.active_animations.erase(
        std::remove_if(springState.active_animations.begin(), springState.active_animations.end(),
                       [](const AdvancedSpringState::ActiveAnimation &anim)
                       {
                           return !anim.is_looping && anim.progress >= 1.0f;
                       }),
        springState.active_animations.end());

    // Limit concurrent animations
    if (springState.active_animations.size() > (size_t)animSystem.max_concurrent_animations)
    {
        // Sort by priority and keep highest priority animations
        std::sort(springState.active_animations.begin(), springState.active_animations.end(),
                  [](const auto &a, const auto &b)
                  { return a.priority > b.priority; });
        springState.active_animations.resize(animSystem.max_concurrent_animations);
    }
}

void Jelly::updateSpringStrengthModulation(uint64_t owner_id, float dt)
{
    if (!gangBeastsSettings->advanced_spring_system.spring_strength_modulation.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];
    const auto &strengthMod = gangBeastsSettings->advanced_spring_system.spring_strength_modulation;

    // Clear previous overrides
    springState.spring_strength_overrides.clear();

    // Apply strength factors based on current state and actions
    float legStrength = 1.0f;  // Default (original behavior)
    float armStrength = 1.0f;  // Default (original behavior)
    float coreStrength = 1.0f; // Default (original behavior)

    switch (springState.current_state)
    {
    case PhysicsState::WALKING:
        legStrength = strengthMod.action_strength_factors.walking_leg_strength;
        armStrength = strengthMod.action_strength_factors.walking_arm_strength;
        break;
    case PhysicsState::JUMPING:
        legStrength = strengthMod.action_strength_factors.jumping_leg_strength;
        break;
    case PhysicsState::FALLING:
        legStrength = strengthMod.action_strength_factors.landing_leg_strength;
        break;
    case PhysicsState::GRABBING:
        armStrength = strengthMod.action_strength_factors.grabbing_arm_strength;
        break;
    case PhysicsState::WAVING:
        armStrength = strengthMod.action_strength_factors.waving_arm_strength;
        break;
    default:
        coreStrength = strengthMod.action_strength_factors.balancing_core_strength;
        break;
    }

    // Apply strength modulation to relevant springs
    for (size_t i = 0; i < springs.size(); ++i)
    {
        if (springs[i].owner_id != owner_id)
            continue;

        // Determine spring type and apply appropriate strength
        // This is a simplified approach - in a full implementation,
        // you'd categorize springs by body region
        if (springs[i].is_skeleton)
        {
            // For now, apply the current stiffness multiplier from context-aware system
            springState.spring_strength_overrides[i] = springState.current_stiffness_multiplier;
        }
    }
}

void Jelly::updateAdaptivePhysics(uint64_t owner_id, float dt)
{
    if (!gangBeastsSettings->advanced_spring_system.adaptive_physics.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];
    auto &adaptiveState = springState.adaptive_state;
    const auto &adaptiveSettings = gangBeastsSettings->advanced_spring_system.adaptive_physics;

    // Update stress response if enabled
    if (adaptiveSettings.stress_response.enabled)
    {
        // Decay stress over time
        adaptiveState.stress_level *= std::pow(0.5f, dt / adaptiveSettings.stress_response.recovery_time);

        // Apply stress weakening (but only if learning rate > 0)
        if (adaptiveSettings.learning_rate > 0.0f && adaptiveState.stress_level > adaptiveSettings.stress_response.stress_threshold)
        {
            // In a full implementation, this would weaken springs under stress
            // For now, we maintain original behavior by not applying weakening
        }
    }

    // Update adaptation memory
    adaptiveState.last_adaptation_time += dt;

    // Decay learned adaptations over time
    if (adaptiveState.last_adaptation_time > adaptiveSettings.adaptation_memory)
    {
        for (auto &[springIndex, learnedStiffness] : adaptiveState.learned_stiffness)
        {
            // Gradually return to original values
            learnedStiffness += (1.0f - learnedStiffness) * dt * 0.1f;
        }
    }
}

// State detection methods
Jelly::PhysicsState Jelly::detectPhysicsState(uint64_t owner_id) const
{
    // Simple state detection based on physics properties
    // In a full implementation, this would analyze velocity, ground contact, etc.

    // For now, default to IDLE to maintain original behavior
    return PhysicsState::IDLE;
}

bool Jelly::isWalking(uint64_t owner_id) const
{
    // Placeholder - would analyze horizontal velocity and ground contact
    return false;
}

bool Jelly::isJumping(uint64_t owner_id) const
{
    // Placeholder - would analyze upward velocity
    return false;
}

bool Jelly::isFalling(uint64_t owner_id) const
{
    // Placeholder - would analyze downward velocity and ground distance
    return false;
}

bool Jelly::isGrabbing(uint64_t owner_id) const
{
    // Placeholder - would check for grab actions
    return false;
}

// Animation system methods
void Jelly::startSpringAnimation(uint64_t owner_id, const std::string &animationName, float duration, int priority, bool looping)
{
    if (!gangBeastsSettings->advanced_spring_system.spring_animation_system.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];

    // Check if animation already exists
    for (auto &anim : springState.active_animations)
    {
        if (anim.name == animationName)
        {
            // Update existing animation
            anim.duration = duration;
            anim.priority = priority;
            anim.is_looping = looping;
            anim.progress = 0.0f; // Restart
            return;
        }
    }

    // Add new animation
    AdvancedSpringState::ActiveAnimation newAnim;
    newAnim.name = animationName;
    newAnim.duration = duration;
    newAnim.priority = priority;
    newAnim.is_looping = looping;
    newAnim.progress = 0.0f;

    springState.active_animations.push_back(newAnim);
}

void Jelly::stopSpringAnimation(uint64_t owner_id, const std::string &animationName)
{
    auto &springState = advancedSpringStates[owner_id];

    springState.active_animations.erase(
        std::remove_if(springState.active_animations.begin(), springState.active_animations.end(),
                       [&animationName](const auto &anim)
                       { return anim.name == animationName; }),
        springState.active_animations.end());
}

float Jelly::getAnimationStrengthMultiplier(uint64_t owner_id, const std::string &bodyRegion) const
{
    // Placeholder - would return strength multiplier based on active animations
    // For now, return 1.0 to maintain original behavior
    return 1.0f;
}

// Adaptive physics methods
void Jelly::applyAdaptivePhysics(uint64_t owner_id, int springIndex, float &stiffness)
{
    if (!gangBeastsSettings->advanced_spring_system.adaptive_physics.enabled)
        return;
    if (gangBeastsSettings->advanced_spring_system.adaptive_physics.learning_rate <= 0.0f)
        return;

    auto &springState = advancedSpringStates[owner_id];
    auto &adaptiveState = springState.adaptive_state;

    // Apply learned stiffness if available
    auto it = adaptiveState.learned_stiffness.find(springIndex);
    if (it != adaptiveState.learned_stiffness.end())
    {
        stiffness *= it->second;
    }
}

void Jelly::recordSpringStress(uint64_t owner_id, int springIndex, float stress)
{
    if (!gangBeastsSettings->advanced_spring_system.adaptive_physics.enabled)
        return;
    if (!gangBeastsSettings->advanced_spring_system.adaptive_physics.stress_response.enabled)
        return;

    auto &springState = advancedSpringStates[owner_id];
    auto &adaptiveState = springState.adaptive_state;

    // Accumulate stress
    adaptiveState.stress_level = std::max(adaptiveState.stress_level, stress);
    adaptiveState.accumulated_forces += stress;

    // For now, don't modify learned stiffness to maintain original behavior
    // In full implementation, this would adapt spring properties based on stress
}
