#pragma once
#include "libs.hpp"

enum class FORCE_TYPE
{
    GLOBAL_FORCE,
    LOCAL_FORCE,
    IMPULSE_FORCE
};
struct Point
{
    sf::Vector2f pos, prev_pos, acc = {0.f, 0.f};

    bool locked = false;
    float mass = 0.0001f;
};
struct Spring
{
    int p1, p2;
    float rest_length;
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
    int iterations = 10;
    // global stiffness multiplier for constraint correction (0..1)
    float stiffness = 0.6f;
    // separate multipliers
    float stiffness_ring = 0.6f;   // peripheral-peripheral springs
    float stiffness_spoke = 0.65f; // center-peripheral springs
    // internal pressure (0 = off). Positive -> expansion force
    float pressure = 1.8f;
    // damping multiplier applied to velocity (0..1, closer to 1 more bouncy)
    float damping = 0.98f;
    // target area for pressure preservation
    float targetArea = 0.f;
    void update(float dt);
    void apply_force(const sf::Vector2f &F_total);
    void apply_force(const sf::Vector2f &F_total, const sf::Vector2f &at, float radius);
    void apply_force(const sf::Vector2f &J_total, const sf::Vector2f &at, float radius, float dt);
    void draw(sf::RenderWindow &window);

    // shape creation helpers
    void clear();
    void create_circle(int n, float r, sf::Vector2f c);
    void create_from_points(const std::vector<sf::Vector2f> &pts);
    // create from points but resample edges to have approximately target peripheral points
    void create_from_points_resampled(const std::vector<sf::Vector2f> &pts, int targetPerimeterPoints);
    // create a filled soft-body by sampling the interior of a polygon with a triangular lattice
    // polygon: ordered peripheral points
    // spacing: approximate distance between interior lattice points (pixels)
    void create_filled_from_polygon(const std::vector<sf::Vector2f> &pts, float spacing = 12.f);

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

    for (int i = 1; i <= N; ++i)
    {
        float angle = i * 2.f * M_PI / float(N);
        sf::Vector2f p = center + sf::Vector2f(std::cos(angle), std::sin(angle)) * radius;
        points[i].pos = p;
        points[i].prev_pos = p;
        points[i].mass = 1.f;
        points[i].locked = false;
    }

    // create springs: ring + center spokes
    springs.clear();
    int centerIndex = 0;
    for (int i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1; // wraps to 1
        sf::Vector2f delta = points[next].pos - points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({i, next, dist});

        sf::Vector2f delta2 = points[i].pos - points[centerIndex].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({centerIndex, i, dist2});
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
        springs.push_back({i, next, dist});

        sf::Vector2f delta2 = points[i].pos - points[0].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({0, i, dist2});
    }
    // compute and store target area for pressure (perimeter only)
    targetArea = compute_area(points, N);
}

void Jelly::create_from_points_resampled(const std::vector<sf::Vector2f> &pts, int targetPerimeterPoints)
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
        sf::Vector2f d = pts[j] - pts[i];
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
            res.push_back(pts[i] + (pts[j] - pts[i]) * t);
        }
    }

    // ensure we have at least targetPerimeterPoints
    while ((int)res.size() < targetPerimeterPoints)
    {
        res.push_back(pts[0]);
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
        springs.push_back({i, next, dist});

        sf::Vector2f delta2 = points[i].pos - points[0].pos;
        float dist2 = std::sqrt(delta2.x * delta2.x + delta2.y * delta2.y);
        springs.push_back({0, i, dist2});
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
                    springs.push_back({idx, j, std::sqrt(d.x * d.x + d.y * d.y)});
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
                        springs.push_back({idx, j, std::sqrt(d.x * d.x + d.y * d.y)});
                    }
                }
                int cj2 = cidx + (r % 2 ? 1 : 0);
                if (cj2 >= 0 && cj2 < cols)
                {
                    int j2 = grid[r + 1][cj2];
                    if (j2 != -1)
                    {
                        sf::Vector2f d = points[j2].pos - points[idx].pos;
                        springs.push_back({idx, j2, std::sqrt(d.x * d.x + d.y * d.y)});
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
                springs.push_back({pi, (int)qi, d});
            }
        }
    }

    // set perimeter count and compute and store target area (use polygon area from perimeter)
    N = Nperim;
    targetArea = compute_area(points, N);
    // set radius for heuristic
    radius = maxr;
}

void Jelly::update_verlet(float dt, sf::Vector2f acceleration)
{
    const sf::Vector2f gravity{0.f, 981.f};

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
            if (dist == 0.f)
                continue;
            float diff = (dist - s.rest_length) / dist;
            float localStiff = (s.p1 == 0 || s.p2 == 0) ? stiffness_spoke : stiffness_ring;
            float corr = 0.5f * diff * stiffness * localStiff; // apply global and local stiffness
            if (!p1.locked)
                p1.pos += delta * corr;
            if (!p2.locked)
                p2.pos -= delta * corr;
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
