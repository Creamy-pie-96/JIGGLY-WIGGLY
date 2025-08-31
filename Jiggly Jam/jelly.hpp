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
    float mass = 1.f;
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
    int iterations = 6;
    // global stiffness multiplier for constraint correction (0..1)
    float stiffness = 1.f;
    // separate multipliers
    float stiffness_ring = 1.f;  // peripheral-peripheral springs
    float stiffness_spoke = 1.f; // center-peripheral springs
    // internal pressure (0 = off). Positive -> expansion force
    float pressure = 0.f;
    // damping multiplier applied to velocity (0..1, closer to 1 more bouncy)
    float damping = 0.99f;
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

// compute polygon area (peripheral points 1..N)
static float compute_area(const std::vector<Point> &points)
{
    int n = (int)points.size() - 1;
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
    // compute and store target area for pressure
    targetArea = compute_area(points);
}

void Jelly::update_verlet(float dt, sf::Vector2f acceleration)
{
    const sf::Vector2f gravity{0.f, 981.f};

    // apply pressure force as acceleration on peripheral points
    if (pressure != 0.f && targetArea > 0.f)
    {
        float currentArea = compute_area(points);
        if (currentArea > 1e-6f)
        {
            float areaRatio = currentArea / targetArea;
            // pressureFactor: positive if current < target (pull outward)
            float pressureFactor = (1.f - areaRatio) * pressure;
            for (int i = 1; i <= (int)points.size() - 1; ++i)
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
    {
        // fallback: if no weighted points (small radius or numerical issue),
        // apply whole impulse to center point if available to ensure a response
        if (!points.empty() && !points[0].locked)
        {
            // apply equivalent acceleration to center point
            sf::Vector2f a = F_total / points[0].mass;
            points[0].acc += a;
        }
        return;
    }

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
        // encode velocity change in Verlet prev_pos (Δv)
        points[i].prev_pos -= dv;
    }
}

void Jelly::draw(sf::RenderWindow &window)
{
    if (points.size() <= 1)
        return;

    // draw filled shape from peripheral points (1..N)
    int peripheralCount = (int)points.size() - 1;
    if (peripheralCount >= 3)
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
