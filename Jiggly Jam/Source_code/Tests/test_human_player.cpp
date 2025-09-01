#include <SFML/Graphics.hpp>
#include <vector>
#include <map>
#include <cmath>
#include "../../Libraries/Gmae_physics/jelly.hpp"

// This helper function builds the human figure by adding points and springs to the Jelly object.
// We are passing in a reference to the Jelly object, so we are modifying the object directly.
void createHumanFigure(Jelly &figure, const sf::Vector2f &origin, const float &height)
{
    float s = height / 175.0f; // Scale factor based on a standard height

    // Define all the points for the skeleton and add them to the figure
    // These points represent the joints and endpoints of the body
    figure.add_point(origin + sf::Vector2f(0.0f, -60.0f) * s, BODY_PART::HEAD);
    figure.add_point(origin + sf::Vector2f(0.0f, -45.0f) * s, BODY_PART::NECK);
    figure.add_point(origin + sf::Vector2f(20.0f, -30.0f) * s, BODY_PART::SHO_R);
    figure.add_point(origin + sf::Vector2f(-20.0f, -30.0f) * s, BODY_PART::SHO_L);
    figure.add_point(origin + sf::Vector2f(35.0f, 0.0f) * s, BODY_PART::ELB_R);
    figure.add_point(origin + sf::Vector2f(-35.0f, 0.0f) * s, BODY_PART::ELB_L);
    figure.add_point(origin + sf::Vector2f(45.0f, 30.0f) * s, BODY_PART::HAND_R);
    figure.add_point(origin + sf::Vector2f(-45.0f, 30.0f) * s, BODY_PART::HAND_L);
    figure.add_point(origin + sf::Vector2f(10.0f, 40.0f) * s, BODY_PART::HIP_R);
    figure.add_point(origin + sf::Vector2f(-10.0f, 40.0f) * s, BODY_PART::HIP_L);
    figure.add_point(origin + sf::Vector2f(15.0f, 80.0f) * s, BODY_PART::KNEE_R);
    figure.add_point(origin + sf::Vector2f(-15.0f, 80.0f) * s, BODY_PART::KNEE_L);
    figure.add_point(origin + sf::Vector2f(20.0f, 105.0f) * s, BODY_PART::FOOT_R);
    figure.add_point(origin + sf::Vector2f(-20.0f, 105.0f) * s, BODY_PART::FOOT_L);

    // Add skeleton springs to connect the points, which define the structure of the figure
    figure.add_edge(BODY_PART::HEAD, BODY_PART::NECK);
    figure.add_edge(BODY_PART::NECK, BODY_PART::SHO_R);
    figure.add_edge(BODY_PART::NECK, BODY_PART::SHO_L);
    figure.add_edge(BODY_PART::SHO_R, BODY_PART::ELB_R);
    figure.add_edge(BODY_PART::ELB_R, BODY_PART::HAND_R);
    figure.add_edge(BODY_PART::SHO_L, BODY_PART::ELB_L);
    figure.add_edge(BODY_PART::ELB_L, BODY_PART::HAND_L);
    figure.add_edge(BODY_PART::NECK, BODY_PART::HIP_R);
    figure.add_edge(BODY_PART::NECK, BODY_PART::HIP_L);
    figure.add_edge(BODY_PART::HIP_R, BODY_PART::HIP_L);
    figure.add_edge(BODY_PART::HIP_R, BODY_PART::KNEE_R);
    figure.add_edge(BODY_PART::HIP_L, BODY_PART::KNEE_L);
    figure.add_edge(BODY_PART::KNEE_R, BODY_PART::FOOT_R);
    figure.add_edge(BODY_PART::KNEE_L, BODY_PART::FOOT_L);
    figure.add_edge(BODY_PART::SHO_R, BODY_PART::HIP_R);
    figure.add_edge(BODY_PART::SHO_L, BODY_PART::HIP_L);
    figure.add_edge(BODY_PART::SHO_R, BODY_PART::HIP_L);
    figure.add_edge(BODY_PART::SHO_L, BODY_PART::HIP_R);
}

// Function to draw the jelly figure with thick lines and a head
void drawJellyFigure(sf::RenderWindow &window, const Jelly &figure)
{
    // Draw the torso as a filled rectangle
    sf::RectangleShape torso({10.f, 60.f});
    torso.setOrigin({5.f, 30.f});
    torso.setPosition(figure.points[figure.part_index.at(BODY_PART::NECK)].pos);
    torso.setRotation(90.f);
    torso.setFillColor(sf::Color(100, 100, 255));
    window.draw(torso);

    // Draw limbs and head as thick lines or rectangles
    for (const auto &s : figure.springs)
    {
        sf::Vector2f p1 = figure.points[s.p1].pos;
        sf::Vector2f p2 = figure.points[s.p2].pos;

        sf::Vector2f d = p2 - p1;
        float length = std::sqrt(d.x * d.x + d.y * d.y);
        float angle = std::atan2(d.y, d.x) * 180.0f / M_PI;

        sf::RectangleShape line({length, 8.0f});
        line.setOrigin({0.0f, 4.0f});
        line.setPosition(p1);
        line.setRotation(angle);
        line.setFillColor(sf::Color::White);
        window.draw(line);
    }

    // Draw the head as a filled circle
    sf::CircleShape head(15.f);
    head.setOrigin({15.f, 15.f});
    head.setPosition(figure.points[figure.part_index.at(BODY_PART::HEAD)].pos);
    head.setFillColor(sf::Color(255, 255, 255));
    window.draw(head);
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Jelly Human Figure");
    window.setFramerateLimit(60);

    Jelly myHumanFigure;
    createHumanFigure(myHumanFigure, sf::Vector2f(400.f, 300.f), 175.f);

    // Lock the feet to the ground so the figure doesn't fall
    myHumanFigure.points[myHumanFigure.part_index.at(BODY_PART::FOOT_R)].locked = true;
    myHumanFigure.points[myHumanFigure.part_index.at(BODY_PART::FOOT_L)].locked = true;

    sf::Clock clock;
    float groundLevel = 500.f; // The ground is at 500 pixels on the y-axis
    bool onGround = false;

    while (window.isOpen())
    {
        sf::Time dt = clock.restart();

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            // Handle jump input (Spacebar press)
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
            {
                // Check if the figure is on the ground before allowing it to jump
                if (onGround)
                {
                    // Apply an upward impulse force to the torso
                    myHumanFigure.apply_force({0.f, -100.f}, myHumanFigure.points[myHumanFigure.part_index.at(BODY_PART::NECK)].pos, 50.f, dt.asSeconds());
                    onGround = false; // The figure is now in the air
                }
            }
        }

        // Apply a downward force (gravity)
        myHumanFigure.apply_force({0.f, 9.8f * 10.f});

        // Update the figure's physics
        myHumanFigure.update(dt.asSeconds());

        // Check for ground collision and reset `onGround` flag
        onGround = false;
        for (auto &p : myHumanFigure.points)
        {
            if (p.pos.y >= groundLevel)
            {
                p.pos.y = groundLevel;
                p.prev_pos.y = p.pos.y;
                onGround = true; // At least one point is touching the ground
            }
        }

        window.clear(sf::Color(20, 20, 20));
        drawJellyFigure(window, myHumanFigure);
        window.display();
    }

    return 0;
}
