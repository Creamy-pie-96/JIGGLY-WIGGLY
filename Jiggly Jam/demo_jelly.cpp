#include "jelly.hpp"
#include "libs.hpp"
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>

int main()
{
    const int W = 1000, H = 700;
    sf::RenderWindow window(sf::VideoMode(W, H), "Jelly Demo - human-ish shape");
    window.setFramerateLimit(60);

    // create a human-ish outline (ordered peripheral points)
    std::vector<sf::Vector2f> human;
    sf::Vector2f origin(500.f, 250.f);
    float scale = 1.0f;
    // head (top)
    human.push_back(origin + sf::Vector2f(-18, -70) * scale);
    human.push_back(origin + sf::Vector2f(18, -70) * scale);
    // shoulders
    human.push_back(origin + sf::Vector2f(36, -30) * scale);
    human.push_back(origin + sf::Vector2f(36, 0) * scale);
    // right hip
    human.push_back(origin + sf::Vector2f(18, 40) * scale);
    human.push_back(origin + sf::Vector2f(6, 80) * scale);
    // right foot
    human.push_back(origin + sf::Vector2f(2, 95) * scale);
    human.push_back(origin + sf::Vector2f(-6, 95) * scale);
    // left foot
    human.push_back(origin + sf::Vector2f(-10, 80) * scale);
    human.push_back(origin + sf::Vector2f(-28, 40) * scale);
    // left arm
    human.push_back(origin + sf::Vector2f(-36, 0) * scale);
    human.push_back(origin + sf::Vector2f(-36, -30) * scale);

    Jelly jelly;
    jelly.create_from_points(human);

    // tune masses and locked points lightly (lock top of head for stability test)
    if (jelly.points.size() > 1)
    {
        // make center slightly heavier
        jelly.points[0].mass = 2.f;
        // increase peripheral mass a bit
        for (size_t i = 1; i < jelly.points.size(); ++i)
            jelly.points[i].mass = 1.0f;
    }

    // ground line
    sf::RectangleShape ground(sf::Vector2f((float)W, 50.f));
    ground.setPosition(0.f, H - 50.f);
    ground.setFillColor(sf::Color(100, 200, 100));

    // fixed timestep
    const float dt = 1.f / 120.f; // physics timestep
    float accumulator = 0.f;
    sf::Clock clock;

    // simple controls
    const float move_force = 20000.f;
    const float jump_impulse = 800.f;

    while (window.isOpen())
    {
        sf::Event ev;
        while (window.pollEvent(ev))
        {
            if (ev.type == sf::Event::Closed)
                window.close();
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Escape)
                window.close();
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Space)
            {
                // apply upward impulse at center
                jelly.apply_force(sf::Vector2f(0.f, -jump_impulse), jelly.points[0].pos, jelly.get_radius() * 1.5f, dt);
            }

            // adjust stiffness using + and - (handle keypad + and regular)
            if (ev.type == sf::Event::KeyPressed && (ev.key.code == sf::Keyboard::Add || ev.key.code == sf::Keyboard::Equal))
            {
                jelly.stiffness = std::min(1.f, jelly.stiffness + 0.05f);
                std::cout << "stiffness = " << jelly.stiffness << std::endl;
            }
            if (ev.type == sf::Event::KeyPressed && (ev.key.code == sf::Keyboard::Subtract || ev.key.code == sf::Keyboard::Hyphen))
            {
                jelly.stiffness = std::max(0.f, jelly.stiffness - 0.05f);
                std::cout << "stiffness = " << jelly.stiffness << std::endl;
            }

            // pressure controls: [ decrease, ] increase
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::LBracket)
            {
                jelly.pressure = std::max(0.f, jelly.pressure - 0.025f);
                std::cout << "pressure = " << jelly.pressure << std::endl;
            }
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::RBracket)
            {
                jelly.pressure = std::min(5.f, jelly.pressure + 0.025f);
                std::cout << "pressure = " << jelly.pressure << std::endl;
            }

            // ring stiffness: 1 increase, 2 decrease
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Num1)
            {
                jelly.stiffness_ring = std::min(1.f, jelly.stiffness_ring + 0.05f);
                std::cout << "stiffness_ring = " << jelly.stiffness_ring << std::endl;
            }
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Num2)
            {
                jelly.stiffness_ring = std::max(0.f, jelly.stiffness_ring - 0.05f);
                std::cout << "stiffness_ring = " << jelly.stiffness_ring << std::endl;
            }

            // spoke stiffness: 3 increase, 4 decrease
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Num3)
            {
                jelly.stiffness_spoke = std::min(1.f, jelly.stiffness_spoke + 0.05f);
                std::cout << "stiffness_spoke = " << jelly.stiffness_spoke << std::endl;
            }
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Num4)
            {
                jelly.stiffness_spoke = std::max(0.f, jelly.stiffness_spoke - 0.05f);
                std::cout << "stiffness_spoke = " << jelly.stiffness_spoke << std::endl;
            }

            // damping: D increase, F decrease
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::D)
            {
                jelly.damping = std::min(0.999f, jelly.damping + 0.005f);
                std::cout << "damping = " << jelly.damping << std::endl;
            }
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::F)
            {
                jelly.damping = std::max(0.9f, jelly.damping - 0.005f);
                std::cout << "damping = " << jelly.damping << std::endl;
            }
        }

        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        // input forces
        sf::Vector2f inputForce{0.f, 0.f};
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            inputForce.x -= move_force;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            inputForce.x += move_force;

        // step physics fixed timesteps
        while (accumulator >= dt)
        {
            // apply global lateral force as local at center so it nudges whole body
            if (inputForce.x != 0.f)
                jelly.apply_force(inputForce * dt, jelly.points[0].pos, jelly.get_radius() * 2.f);

            // continuous jump while holding Space: apply an upward local force
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            {
                const float hold_jump_force = 120000.f; // tune this value
                jelly.apply_force(sf::Vector2f(0.f, -hold_jump_force), jelly.points[0].pos, jelly.get_radius() * 1.2f);
            }

            jelly.update(dt);
            // simple ground collision: project any point below ground up
            for (auto &p : jelly.points)
            {
                float minY = H - 50.f - 1.f; // just above ground
                if (p.pos.y > minY)
                {
                    p.pos.y = minY;
                    // damp velocity
                    p.prev_pos.y = p.pos.y + (p.prev_pos.y - p.pos.y) * 0.3f;
                }
            }

            accumulator -= dt;
        }

        window.clear(sf::Color(30, 30, 40));
        window.draw(ground);
        jelly.draw(window);

        // instructions
        // (avoid font dependency; simple indicator via console once)
        static bool shown = false;
        if (!shown)
        {
            std::cout << "Controls: Left/Right to nudge, Space to jump, Esc to quit\n";
            shown = true;
        }

        window.display();
    }

    return 0;
}