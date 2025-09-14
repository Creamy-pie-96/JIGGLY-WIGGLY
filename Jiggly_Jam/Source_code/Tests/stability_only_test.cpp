#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"

int main()
{
    const int W = 1200, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "Stability Only Test - No Controls");
    window.setFramerateLimit(60);

    Player player("test", &window);

    float ground_y = H - 100.0f;
    player.set_ground_level(ground_y);

    player.spawn({W / 2.0f, 200.0f});

    // DISABLE advanced controls to test pure stability
    player.setAdvancedControls(false);

    sf::Clock clock;
    float accumulator = 0.0f;
    const float dt = 1.0f / 120.0f; // Same timestep as debug_springs_test

    std::cout << "\n🧪 === STABILITY ONLY TEST ===" << std::endl;
    std::cout << "💡 Testing if figure stands stable WITHOUT any controls" << std::endl;
    std::cout << "📊 Initial position: (" << player.get_position().x << ", " << player.get_position().y << ")" << std::endl;

    sf::Clock testClock;
    float testDuration = 10.0f; // 10 seconds

    while (window.isOpen() && testClock.getElapsedTime().asSeconds() < testDuration)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
                break;
            }
        }

        // Fixed timestep physics - NO INPUT, JUST STABILITY
        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        while (accumulator >= dt)
        {
            player.update(dt); // Should only apply postural stability
            accumulator -= dt;
        }

        window.clear(sf::Color(40, 50, 60));

        sf::RectangleShape ground(sf::Vector2f(W, 100));
        ground.setPosition(0, H - 100);
        ground.setFillColor(sf::Color(100, 150, 100));
        window.draw(ground);

        auto figure = player.Figure();

        for (const auto &spring : figure.springs)
        {
            if (spring.p1 >= 0 && spring.p1 < (int)figure.points.size() &&
                spring.p2 >= 0 && spring.p2 < (int)figure.points.size())
            {
                sf::Vector2f p1 = figure.points[spring.p1].pos;
                sf::Vector2f p2 = figure.points[spring.p2].pos;

                sf::Vertex line[] = {
                    sf::Vertex(p1, spring.is_skeleton ? sf::Color::Red : sf::Color(100, 100, 100, 100)),
                    sf::Vertex(p2, spring.is_skeleton ? sf::Color::Red : sf::Color(100, 100, 100, 100))};
                window.draw(line, 2, sf::Lines);
            }
        }

        for (const auto &point : figure.points)
        {
            sf::CircleShape dot(4.0f);
            dot.setPosition(point.pos.x - 4.0f, point.pos.y - 4.0f);

            // Color code by body part
            if (point.body_part == BODY_PART::HEAD)
                dot.setFillColor(sf::Color::Yellow);
            else if (point.body_part == BODY_PART::FOOT_L || point.body_part == BODY_PART::FOOT_R)
                dot.setFillColor(sf::Color::Green);
            else if (point.body_part == BODY_PART::HAND_L || point.body_part == BODY_PART::HAND_R)
                dot.setFillColor(sf::Color::Blue);
            else if (point.id == player.get_id())
                dot.setFillColor(sf::Color::Red); // Skeleton
            else
                dot.setFillColor(sf::Color::Cyan); // Flesh

            window.draw(dot);
        }

        window.display();
    }

    // Final analysis
    sf::Vector2f finalPos = player.get_position();
    std::cout << "📊 Final position: (" << finalPos.x << ", " << finalPos.y << ")" << std::endl;
    std::cout << "🧪 Stability Test Result: " << (finalPos.y > 600 ? "✅ STABLE (grounded)" : "❌ COLLAPSED") << std::endl;

    return 0;
}
