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
    float mass;
};
struct Spring
{
    int p1, p2;
    float rest_length;
};

class Jelly
{
private:
    /* data */
    int N = 20;
    float radius = 30;
    sf::Vector2f center{200.f, 300.f};
    void update_verlet(float dt, sf::Vector2f acceleration = {0, 0});

public:
    Jelly(/* args */);
    ~Jelly();

    std::vector<Point> points;
    std::vector<Spring> springs;
    float radius;
    int itearations=6;
    void update(float dt);
    void apply_force(const sf::Vector2f &F_total);
    void apply_force(const sf::Vector2f &F_total, const sf::Vector2f &at, float radius);
    void apply_force(const sf::Vector2f &J_total, const sf::Vector2f &at, float radius, float dt);
    void draw(sf::RenderWindow &window);

    Jelly(int n, float r = 12, sf::Vector2f c);
};

Jelly::Jelly(/* args */)
{
    this->points.push_back({center, center, {0, 0}, false}); // center not locked
    this->points.resize(N + 1);
    for (size_t i = 1; i <= N; ++i)
    {
        float angle = i * 2 * M_PI / N;
        this->points[i].pos = center + sf::Vector2f(cos(angle), sin(angle)) * radius;
        this->points[i].prev_pos = this->points[i].pos;
    }
    int centerIndex = 0;
    for (size_t i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1;
        sf::Vector2f delta = this->points[next].pos - this->points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({static_cast<int>(i), static_cast<int>(next), dist});
        sf::Vector2f delta = points[i].pos - points[centerIndex].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({static_cast<int>(centerIndex), static_cast<int>(i), dist});
    }
}

Jelly::~Jelly()
{
}

Jelly ::Jelly(int n, float r, sf::Vector2f c) : N(n), radius(r), center(c)
{
    this->points.push_back({center, center, {0, 0}, false}); // center not locked
    this->points.resize(N + 1);
    for (size_t i = 1; i <= N; ++i)
    {
        float angle = i * 2 * M_PI / N;
        this->points[i].pos = center + sf::Vector2f(cos(angle), sin(angle)) * radius;
        this->points[i].prev_pos = this->points[i].pos;
    }
    int centerIndex = 0;
    for (size_t i = 1; i <= N; ++i)
    {
        int next = (i % N) + 1;
        sf::Vector2f delta = this->points[next].pos - this->points[i].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({static_cast<int>(i), static_cast<int>(next), dist});
        sf::Vector2f delta = points[i].pos - points[centerIndex].pos;
        float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        springs.push_back({static_cast<int>(centerIndex), static_cast<int>(i), dist});
    }
}

void Jelly ::update_verlet(float dt, sf::Vector2f acceleration = {0, 0})
{
    sf::Vector2f velocity;
    sf::Vector2f gravity{0, 981.f};
    float damping = 0.99f;
    // Standard for loop (size_t)
    // Range-based for loop
    
    for (auto &p : points)
    {
        if (p.locked)
            continue;
        acceleration += gravity + p.acc;
        sf::Vector2f velocity = (p.pos - p.prev_pos) * damping;
        sf::Vector2f new_pos = p.pos + velocity + acceleration * dt * dt;
        p.prev_pos = p.pos;
        p.pos = new_pos;
        p.acc={0,0};
    }
    // Standard for loop (size_t)
    for (size_t i = 0; i < itearations; ++i)
    {

        // Range-based for loop
        for (auto &s : springs)
        {
            Point &p1 = points[s.p1];
            Point &p2 = points[s.p2];
            if (p1.locked && p2.locked)
                continue;
            sf::Vector2f delta = p2.pos - p1.pos;
            float dist = sqrt(delta.x * delta.x + delta.y * delta.y);
            float diff = (dist - s.rest_length) / dist;
            if (!p1.locked)
                p1.pos += delta * 0.5f * diff;
            if (!p2.locked)
                p2.pos -= delta * 0.5f * diff;
        }
    }
}

void Jelly::update(float dt)
{
    update_verlet(dt);
}

// TODO i need to review this part later cz i did not totally understand this
// this is global force
void Jelly::apply_force(const sf::Vector2f &F_total)
{
    float M = 0.f;
    for (auto &p : points)
        M += p.mass;
    if (M == 0)
        return;

    // This gives every particle the same acceleration a = F_total / M
    sf::Vector2f a = F_total / M;
    for (auto &p : points)
        p.acc += a; // a = F_total / M = (F_i / m_i) with F_i = (m_i / M) * F_total
}

static float falloff(float d, float R)
{
    if (d >= R)
        return 0.f;
    float x = 1.f - d / R;
    return x * x;
}

// this is local force
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

    // Distribute so Σ F_i = F_total (mass-aware acceleration = F_i / m_i)
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].locked || w[i] == 0.f)
            continue;
        sf::Vector2f Fi = (w[i] / sumw) * F_total;
        points[i].acc += Fi / points[i].mass;
    }
}

// this is for impulse force
void Jelly::apply_force(const sf::Vector2f &J_total, const sf::Vector2f &at, float radius, float dt)
{
    std::vector<float> w(points.size(), 0.f);
    float sumw = 0.f;
    for (size_t i = 0; i < points.size(); ++i)
    {
        float d = hypot(points[i].pos.x - at.x, points[i].pos.y - at.y);
        w[i] = falloff(d, radius);
        sumw += w[i];
    }
    if (sumw <= 1e-6f)
        return;

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (points[i].locked || w[i] == 0.f)
            continue;
        sf::Vector2f Ji = (w[i] / sumw) * J_total; // Σ Ji = J_total
        sf::Vector2f dv = Ji / points[i].mass;     // impulse → Δv
        points[i].prev_pos -= dv * dt;             // encode velocity in Verlet
    }
}
