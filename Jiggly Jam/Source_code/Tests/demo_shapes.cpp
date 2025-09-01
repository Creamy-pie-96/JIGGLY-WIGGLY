#include "../../Libraries/Gmae_physics/jelly.hpp"
#include "../../Libraries/libs.hpp"
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>

int main()
{
    const int W = 1400, H = 700;
    sf::RenderWindow window(sf::VideoMode(W, H), "Jelly Shapes Demo");
    window.setFramerateLimit(60);

    std::vector<Jelly> jellies;
    jellies.resize(4);

    // positions for the four shapes
    std::vector<sf::Vector2f> centers = {
        {250.f, 250.f},
        {550.f, 250.f},
        {850.f, 250.f},
        {1150.f, 250.f}};

    // circle
    jellies[0].create_circle(24, 70.f, centers[0]);
    // triangle (peripheral ordered)
    std::vector<sf::Vector2f> tri = {
        centers[1] + sf::Vector2f(-40.f, -60.f),
        centers[1] + sf::Vector2f(60.f, 0.f),
        centers[1] + sf::Vector2f(-40.f, 60.f),
    };
    jellies[1].create_from_points_resampled(tri, 24);

    // rectangle
    std::vector<sf::Vector2f> rect = {
        centers[2] + sf::Vector2f(-60.f, -40.f),
        centers[2] + sf::Vector2f(60.f, -40.f),
        centers[2] + sf::Vector2f(60.f, 40.f),
        centers[2] + sf::Vector2f(-60.f, 40.f),
    };
    jellies[2].create_from_points_resampled(rect, 24);

    // polygon (star-ish)
    std::vector<sf::Vector2f> poly = {
        centers[3] + sf::Vector2f(-40.f, -70.f),
        centers[3] + sf::Vector2f(0.f, -20.f),
        centers[3] + sf::Vector2f(40.f, -70.f),
        centers[3] + sf::Vector2f(70.f, 0.f),
        centers[3] + sf::Vector2f(40.f, 70.f),
        centers[3] + sf::Vector2f(-40.f, 70.f),
        centers[3] + sf::Vector2f(-70.f, 0.f),
    };
    jellies[3].create_from_points_resampled(poly, 24);

    // default tuning
    for (auto &j : jellies)
    {
        j.stiffness = 0.8f;
        j.stiffness_ring = 0.9f;
        j.stiffness_spoke = 0.9f;
        j.damping = 0.99f;
        j.iterations = 6;
        j.pressure = 0.f;
    }

    // ground
    sf::RectangleShape ground(sf::Vector2f((float)W, 60.f));
    ground.setPosition(0.f, H - 60.f);
    ground.setFillColor(sf::Color(80, 140, 80));

    const float dt = 1.f / 120.f;
    float accumulator = 0.f;
    sf::Clock clock;

    sf::Vector2f inputForce{0.f, 0.f};
    const float move_force = 20000.f;
    const float jump_impulse = 700.f;

    int selected = 0;
    std::cout << "Shapes demo: Select shape with Left/Right arrows. Use Left/Right to nudge, Space to jump.\n";
    std::cout << "Tuning: +/- stiffness, [/] pressure, 1/2 ring stiffness, 3/4 spoke stiffness, D/F damping\n";

    while (window.isOpen())
    {
        sf::Event ev;
        while (window.pollEvent(ev))
        {
            if (ev.type == sf::Event::Closed)
                window.close();
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Escape)
                window.close();

            if (ev.type == sf::Event::KeyPressed)
            {
                if (ev.key.code == sf::Keyboard::Space)
                {
                    jellies[selected].apply_force(sf::Vector2f(0.f, -jump_impulse), jellies[selected].points[0].pos, jellies[selected].get_radius() * 1.5f, dt);
                }

                // select shape with arrow keys
                if (ev.key.code == sf::Keyboard::Right)
                {
                    selected = (selected + 1) % (int)jellies.size();
                    std::cout << "selected " << (selected + 1) << "\n";
                }
                if (ev.key.code == sf::Keyboard::Left)
                {
                    selected = (selected - 1 + (int)jellies.size()) % (int)jellies.size();
                    std::cout << "selected " << (selected + 1) << "\n";
                }

                // tuning keys operate on currently selected jelly
                if (ev.key.code == sf::Keyboard::Add || ev.key.code == sf::Keyboard::Equal)
                {
                    jellies[selected].stiffness = std::min(1.f, jellies[selected].stiffness + 0.05f);
                    std::cout << "stiffness=" << jellies[selected].stiffness << "\n";
                }
                if (ev.key.code == sf::Keyboard::Subtract || ev.key.code == sf::Keyboard::Hyphen)
                {
                    jellies[selected].stiffness = std::max(0.f, jellies[selected].stiffness - 0.05f);
                    std::cout << "stiffness=" << jellies[selected].stiffness << "\n";
                }
                if (ev.key.code == sf::Keyboard::LBracket)
                {
                    jellies[selected].pressure = std::max(0.f, jellies[selected].pressure - 0.025f);
                    std::cout << "pressure=" << jellies[selected].pressure << "\n";
                }
                if (ev.key.code == sf::Keyboard::RBracket)
                {
                    jellies[selected].pressure = std::min(5.f, jellies[selected].pressure + 0.025f);
                    std::cout << "pressure=" << jellies[selected].pressure << "\n";
                }
                // ring stiffness: 1 increase, 2 decrease
                if (ev.key.code == sf::Keyboard::Num1)
                {
                    jellies[selected].stiffness_ring = std::min(1.f, jellies[selected].stiffness_ring + 0.05f);
                    std::cout << "stiffness_ring=" << jellies[selected].stiffness_ring << "\n";
                }
                if (ev.key.code == sf::Keyboard::Num2)
                {
                    jellies[selected].stiffness_ring = std::max(0.f, jellies[selected].stiffness_ring - 0.05f);
                    std::cout << "stiffness_ring=" << jellies[selected].stiffness_ring << "\n";
                }
                // spoke stiffness: 3 increase, 4 decrease
                if (ev.key.code == sf::Keyboard::Num3)
                {
                    jellies[selected].stiffness_spoke = std::min(1.f, jellies[selected].stiffness_spoke + 0.05f);
                    std::cout << "stiffness_spoke=" << jellies[selected].stiffness_spoke << "\n";
                }
                if (ev.key.code == sf::Keyboard::Num4)
                {
                    jellies[selected].stiffness_spoke = std::max(0.f, jellies[selected].stiffness_spoke - 0.05f);
                    std::cout << "stiffness_spoke=" << jellies[selected].stiffness_spoke << "\n";
                }
                // ring/spoke via other keys can be added later
                if (ev.key.code == sf::Keyboard::D)
                {
                    jellies[selected].damping = std::min(0.999f, jellies[selected].damping + 0.005f);
                    std::cout << "damping=" << jellies[selected].damping << "\n";
                }
                if (ev.key.code == sf::Keyboard::F)
                {
                    jellies[selected].damping = std::max(0.9f, jellies[selected].damping - 0.005f);
                    std::cout << "damping=" << jellies[selected].damping << "\n";
                }
            }
        }

        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        inputForce = {0.f, 0.f};
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            inputForce.x -= move_force;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            inputForce.x += move_force;

        while (accumulator >= dt)
        {
            if (inputForce.x != 0.f)
                jellies[selected].apply_force(inputForce * dt, jellies[selected].points[0].pos, jellies[selected].get_radius() * 2.f);

            // continuous jump while holding Space
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            {
                const float hold_jump_force = 120000.f;
                jellies[selected].apply_force(sf::Vector2f(0.f, -hold_jump_force), jellies[selected].points[0].pos, jellies[selected].get_radius() * 1.2f);
            }

            for (auto &j : jellies)
                j.update(dt);

            // ground collision
            for (auto &j : jellies)
            {
                for (auto &p : j.points)
                {
                    float minY = H - 60.f - 1.f;
                    if (p.pos.y > minY)
                    {
                        p.pos.y = minY;
                        p.prev_pos.y = p.pos.y + (p.prev_pos.y - p.pos.y) * 0.3f;
                    }
                }
            }

            accumulator -= dt;
        }

        window.clear(sf::Color(30, 30, 40));
        window.draw(ground);
        for (size_t i = 0; i < jellies.size(); ++i)
        {
            jellies[i].draw(window);
            // highlight selected
            if ((int)i == selected)
            {
                sf::CircleShape s(6.f);
                s.setOrigin(6.f, 6.f);
                s.setFillColor(sf::Color::Yellow);
                s.setPosition(jellies[i].points[0].pos);
                window.draw(s);
            }
        }

        window.display();
    }

    return 0;
}
