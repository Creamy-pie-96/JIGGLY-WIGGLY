#include "jelly.hpp"
#include "libs.hpp"
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
        {1150.f, 250.f}
    };

    // circle
    jellies[0].create_circle(24, 70.f, centers[0]);
    // triangle (peripheral ordered)
    std::vector<sf::Vector2f> tri = {
        centers[1] + sf::Vector2f(-40.f, -60.f),
        centers[1] + sf::Vector2f(60.f, 0.f),
        centers[1] + sf::Vector2f(-40.f, 60.f),
    };
    jellies[1].create_from_points(tri);

    // rectangle
    std::vector<sf::Vector2f> rect = {
        centers[2] + sf::Vector2f(-60.f, -40.f),
        centers[2] + sf::Vector2f(60.f, -40.f),
        centers[2] + sf::Vector2f(60.f, 40.f),
        centers[2] + sf::Vector2f(-60.f, 40.f),
    };
    jellies[2].create_from_points(rect);

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
    jellies[3].create_from_points(poly);

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
    std::cout << "Shapes demo: Select shape with keys 1..4. Use Left/Right/Space to nudge/jump.\n";
    std::cout << "Tuning: +/- stiffness, [/] pressure, Num1/Num2 ring, Num3/Num4 spoke, D/F damping\n";

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

                // select shape with digit keys (top row)
                if (ev.key.code == sf::Keyboard::Num1) { selected = 0; std::cout << "selected 1\n"; }
                if (ev.key.code == sf::Keyboard::Num2) { selected = 1; std::cout << "selected 2\n"; }
                if (ev.key.code == sf::Keyboard::Num3) { selected = 2; std::cout << "selected 3\n"; }
                if (ev.key.code == sf::Keyboard::Num4) { selected = 3; std::cout << "selected 4\n"; }

                // tuning keys operate on currently selected jelly
                if (ev.key.code == sf::Keyboard::Add || ev.key.code == sf::Keyboard::Equal) { jellies[selected].stiffness = std::min(1.f, jellies[selected].stiffness + 0.05f); std::cout << "stiffness="<<jellies[selected].stiffness<<"\n"; }
                if (ev.key.code == sf::Keyboard::Subtract || ev.key.code == sf::Keyboard::Hyphen) { jellies[selected].stiffness = std::max(0.f, jellies[selected].stiffness - 0.05f); std::cout << "stiffness="<<jellies[selected].stiffness<<"\n"; }
                if (ev.key.code == sf::Keyboard::LBracket) { jellies[selected].pressure = std::max(0.f, jellies[selected].pressure - 0.05f); std::cout<<"pressure="<<jellies[selected].pressure<<"\n"; }
                if (ev.key.code == sf::Keyboard::RBracket) { jellies[selected].pressure = std::min(5.f, jellies[selected].pressure + 0.05f); std::cout<<"pressure="<<jellies[selected].pressure<<"\n"; }
                // ring/spoke via other keys can be added later
                if (ev.key.code == sf::Keyboard::D) { jellies[selected].damping = std::min(0.999f, jellies[selected].damping + 0.005f); std::cout<<"damping="<<jellies[selected].damping<<"\n"; }
                if (ev.key.code == sf::Keyboard::F) { jellies[selected].damping = std::max(0.9f, jellies[selected].damping - 0.005f); std::cout<<"damping="<<jellies[selected].damping<<"\n"; }
            }
        }

        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        inputForce = {0.f, 0.f};
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) inputForce.x -= move_force;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) inputForce.x += move_force;

        while (accumulator >= dt)
        {
            if (inputForce.x != 0.f)
                jellies[selected].apply_force(inputForce * dt, jellies[selected].points[0].pos, jellies[selected].get_radius() * 2.f);

            // continuous jump while holding Space: use impulse form for consistent Δv
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            {
                const float hold_jump_impulse = 200.f;
                jellies[selected].apply_force(sf::Vector2f(0.f, -hold_jump_impulse), jellies[selected].points[0].pos, jellies[selected].get_radius() * 1.2f, dt);
            }

            for (auto &j : jellies) j.update(dt);

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
