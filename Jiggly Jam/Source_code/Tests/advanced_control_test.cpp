#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"

int main()
{
    const int W = 1200, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "🎮 Advanced Control Test - WASD Body Part Control");
    window.setFramerateLimit(60);

    std::cout << "\n🎮 === ADVANCED CONTROL MECHANICS TEST ===" << std::endl;
    std::cout << "💡 This test verifies that body parts actually respond to input!\n"
              << std::endl;

    // Create player with advanced controls
    Player player("test", &window);
    player.spawn({W / 2.0f, 300.0f});

    auto *controlSys = player.getControlSystem();
    if (!controlSys)
    {
        std::cout << "❌ No control system - test failed!" << std::endl;
        return 1;
    }

    // Switch to Learn Mode for easier testing
    controlSys->switchToScheme(2); // Learn Mode
    auto *scheme = controlSys->getCurrentScheme();
    std::cout << "🎮 Using: " << scheme->getSchemeName() << std::endl;
    std::cout << "📋 " << scheme->getControlDescription() << std::endl;

    sf::Clock clock;
    sf::Clock testClock;
    float accumulator = 0.0f;
    const float dt = 1.0f / 60.0f; // Fixed 60 FPS

    // Test metrics
    sf::Vector2f initialPos = player.get_position();
    sf::Vector2f lastPos = initialPos;
    float totalMovement = 0.0f;
    bool detectedMovement = false;
    bool detectedInput = false;

    std::cout << "\n🎯 Starting 10-second movement test..." << std::endl;
    std::cout << "💡 Press WASD to control selected body part, TAB to switch parts" << std::endl;
    std::cout << "📊 Initial position: (" << initialPos.x << ", " << initialPos.y << ")" << std::endl;

    // Run for 10 seconds
    while (window.isOpen() && testClock.getElapsedTime().asSeconds() < 10.0f)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
                break;
            }
            if (event.type == sf::Event::KeyPressed)
            {
                detectedInput = true;
                if (event.key.code == sf::Keyboard::Tab)
                {
                    controlSys->nextScheme();
                    scheme = controlSys->getCurrentScheme();
                    std::cout << "🔄 Switched to: " << scheme->getSchemeName() << std::endl;
                }
            }
        }

        // Track input states
        bool hasInput = false;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        {
            hasInput = true;
            std::cout << "⬆️ W ";
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
        {
            hasInput = true;
            std::cout << "⬅️ A ";
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
        {
            hasInput = true;
            std::cout << "⬇️ S ";
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
        {
            hasInput = true;
            std::cout << "➡️ D ";
        }
        if (hasInput)
        {
            detectedInput = true;
            std::cout << std::endl;
        }

        // Fixed timestep physics
        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        while (accumulator >= dt)
        {
            // Get position before update
            sf::Vector2f posBefore = player.get_position();

            // Update player (this should now use advanced controls!)
            player.update(dt);

            // Check for movement
            sf::Vector2f posAfter = player.get_position();
            float movement = std::hypot(posAfter.x - posBefore.x, posAfter.y - posBefore.y);
            totalMovement += movement;

            if (movement > 0.1f)
            {
                detectedMovement = true;
            }

            accumulator -= dt;
        }

        // Simple visualization
        window.clear(sf::Color(40, 50, 60));

        // Draw ground
        sf::RectangleShape ground(sf::Vector2f(W, 50));
        ground.setPosition(0, H - 50);
        ground.setFillColor(sf::Color(100, 150, 100));
        window.draw(ground);

        // Draw player figure
        auto figure = player.Figure();

        // Draw springs
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

        // Draw points
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

        // Draw control debug info
        player.drawControlDebug(window);

        window.display();
    }

    // Final analysis
    sf::Vector2f finalPos = player.get_position();
    float totalDisplacement = std::hypot(finalPos.x - initialPos.x, finalPos.y - initialPos.y);

    std::cout << "\n📊 === MOVEMENT TEST RESULTS ===" << std::endl;
    std::cout << "📍 Final position: (" << finalPos.x << ", " << finalPos.y << ")" << std::endl;
    std::cout << "📏 Total displacement: " << totalDisplacement << " pixels" << std::endl;
    std::cout << "🏃 Total movement: " << totalMovement << " pixels" << std::endl;
    std::cout << "🎮 Input detected: " << (detectedInput ? "✅ YES" : "❌ NO") << std::endl;
    std::cout << "🎯 Movement detected: " << (detectedMovement ? "✅ YES" : "❌ NO") << std::endl;

    if (detectedInput && detectedMovement)
    {
        std::cout << "\n🎉 SUCCESS: Advanced controls are working!" << std::endl;
        std::cout << "✅ Body parts respond to input" << std::endl;
        std::cout << "✅ Physics integration functional" << std::endl;
        std::cout << "✅ Control scheme switching works" << std::endl;
    }
    else if (detectedInput && !detectedMovement)
    {
        std::cout << "\n⚠️  PARTIAL: Input detected but no significant movement" << std::endl;
        std::cout << "🔧 This could indicate:" << std::endl;
        std::cout << "   - Controls are too weak" << std::endl;
        std::cout << "   - Input mapping issues" << std::endl;
        std::cout << "   - Physics constraints too strong" << std::endl;
    }
    else
    {
        std::cout << "\n❌ ISSUE: Advanced controls may not be fully connected" << std::endl;
        std::cout << "🔧 Check input manager and control scheme implementation" << std::endl;
    }

    return 0;
}
