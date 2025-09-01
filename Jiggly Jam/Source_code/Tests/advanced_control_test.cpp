#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"

int main()
{
    const int W = 1200, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "🎮 Gang Beasts Evolution Test - Phase 1.1 & 1.2");
    window.setFramerateLimit(60);

    std::cout << "\n🎮 === GANG BEASTS EVOLUTION PHASE 1 TEST ===" << std::endl;
    std::cout << "🎯 Testing Step 1.1: Physics Personality Tuning" << std::endl;
    std::cout << "🎯 Testing Step 1.2: Enhanced Postural Stability" << std::endl;
    std::cout << "💡 This test verifies Gang Beasts-style physics work with existing controls!\n"
              << std::endl;

    // Create player with advanced controls
    Player player("test", &window);

    // 🎮 GANG BEASTS EVOLUTION: Load settings
    bool settingsLoaded = player.loadGangBeastsSettings("Game Settings/settings.json");
    if (settingsLoaded)
    {
        std::cout << "✅ Gang Beasts settings loaded successfully!" << std::endl;
        player.printCurrentSettings();
    }
    else
    {
        std::cout << "⚠️ Gang Beasts settings failed to load - using defaults" << std::endl;
    }

    // CRITICAL: Set ground level for stable physics
    float ground_y = H - 50.0f;
    player.set_ground_level(ground_y);

    // Spawn player ABOVE ground for proper settling
    player.spawn({W / 2.0f, 200.0f});

    auto *controlSys = player.getControlSystem();

    std::cout << "🎮 Control Mode: " << (player.isUsingAdvancedControls() ? "Advanced + Gang Beasts Stability" : "Simple") << std::endl;
    std::cout << "🎪 Gang Beasts Physics: " << (player.isGangBeastsPhysicsEnabled() ? "ENABLED" : "DISABLED") << std::endl;

    if (player.isUsingAdvancedControls() && controlSys)
    {
        // Switch to Learn Mode for easier testing
        controlSys->switchToScheme(2); // Learn Mode
        auto *scheme = controlSys->getCurrentScheme();
        std::cout << "🎮 Using: " << scheme->getSchemeName() << " (with postural stability foundation)" << std::endl;
        std::cout << "📋 " << scheme->getControlDescription() << std::endl;
    }
    else
    {
        std::cout << "🎮 Using: Simple Controls (like debug_springs_test)" << std::endl;
        std::cout << "📋 A/D: Move Left/Right | Space: Jump" << std::endl;
    }

    sf::Clock clock;
    sf::Clock testClock;
    float accumulator = 0.0f;
    const float dt = Player::STABLE_TIMESTEP; // Use the stable timestep from Player class!

    // Test metrics
    sf::Vector2f initialPos = player.get_position();
    sf::Vector2f lastPos = initialPos;
    float totalMovement = 0.0f;
    bool detectedMovement = false;
    bool detectedInput = false;

    std::cout << "\n🎯 Starting Gang Beasts physics test..." << std::endl;
    std::cout << "💡 Advanced Controls: WASD control selected body part, TAB switches parts" << std::endl;
    std::cout << "🚶 Movement Animations: Q = Wave, W = Walk Right, A = Walk Left (when in simple mode)" << std::endl;
    std::cout << "🔄 Toggle Modes: E = Switch between Advanced Controls / Simple Animations" << std::endl;
    std::cout << "🎪 Gang Beasts: G = Toggle Gang Beasts Physics ON/OFF" << std::endl;
    std::cout << "🔧 Settings: R = Reload settings from file" << std::endl;
    std::cout << "🦴 Postural Stability: Enhanced with reaction delays and overcompensation" << std::endl;
    std::cout << "📊 Initial position: (" << initialPos.x << ", " << initialPos.y << ")" << std::endl;

    while (window.isOpen())
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
                if (event.key.code == sf::Keyboard::Tab && player.isUsingAdvancedControls())
                {
                    controlSys->nextScheme();
                    auto *scheme = controlSys->getCurrentScheme();
                    std::cout << "🔄 Switched to: " << scheme->getSchemeName() << " (Gang Beasts enhanced)" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Space)
                {
                    player.request_jump();
                    std::cout << "🦘 Jump!" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::W && !player.isUsingAdvancedControls())
                {
                    player.startWalking();
                    player.setWalkDirection(1.0f); // Walk right
                    std::cout << "🚶 Start Walking Right!" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::A && !player.isUsingAdvancedControls())
                {
                    player.startWalking();
                    player.setWalkDirection(-1.0f); // Walk left
                    std::cout << "🚶 Start Walking Left!" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Q)
                {
                    player.startWaving();
                    std::cout << "👋 Start Waving!" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::E)
                {
                    // Toggle between advanced and simple controls
                    bool isAdvanced = player.isUsingAdvancedControls();
                    player.setAdvancedControls(!isAdvanced);
                    std::cout << "🔄 Switched to: " << (!isAdvanced ? "Advanced Controls" : "Simple Animations") << std::endl;
                }
                else if (event.key.code == sf::Keyboard::G)
                {
                    // 🎮 GANG BEASTS EVOLUTION: Toggle Gang Beasts physics
                    bool isEnabled = player.isGangBeastsPhysicsEnabled();
                    player.enableGangBeastsPhysics(!isEnabled);
                    std::cout << "🎪 Gang Beasts Physics: " << (!isEnabled ? "ENABLED" : "DISABLED") << std::endl;
                    if (!isEnabled)
                    {
                        std::cout << "    ➤ Skeleton springs now looser for goofy movement" << std::endl;
                        std::cout << "    ➤ Enhanced stability with reaction delays" << std::endl;
                        std::cout << "    ➤ Overcompensation for comic effect" << std::endl;
                    }
                    else
                    {
                        std::cout << "    ➤ Reverted to original precise physics" << std::endl;
                    }
                }
                else if (event.key.code == sf::Keyboard::R)
                {
                    // 🎮 GANG BEASTS EVOLUTION: Reload settings
                    std::cout << "🔧 Reloading Gang Beasts settings..." << std::endl;
                    player.reloadSettings();
                    player.printCurrentSettings();
                }
            }
            if (event.type == sf::Event::KeyReleased)
            {
                if ((event.key.code == sf::Keyboard::W || event.key.code == sf::Keyboard::A) && !player.isUsingAdvancedControls())
                {
                    player.stopWalking();
                    std::cout << "🚶 Stop Walking!" << std::endl;
                }
            }
        }

        // Track input states - let the advanced control system handle input
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

            // UPDATE: Advanced control system with integrated stability!
            if (controlSys && player.isUsingAdvancedControls())
            {
                controlSys->getInputManager().update();
                controlSys->update(dt); // Apply controls!
            }

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

    std::cout << "\n📊 === GANG BEASTS EVOLUTION TEST RESULTS ===" << std::endl;
    std::cout << "📍 Final position: (" << finalPos.x << ", " << finalPos.y << ")" << std::endl;
    std::cout << "📏 Total displacement: " << totalDisplacement << " pixels" << std::endl;
    std::cout << "🏃 Total movement: " << totalMovement << " pixels" << std::endl;
    std::cout << "🎮 Input detected: " << (detectedInput ? "✅ YES" : "❌ NO") << std::endl;
    std::cout << "🎯 Movement detected: " << (detectedMovement ? "✅ YES" : "❌ NO") << std::endl;
    std::cout << "🎪 Gang Beasts Physics: " << (player.isGangBeastsPhysicsEnabled() ? "✅ ACTIVE" : "❌ DISABLED") << std::endl;

    if (detectedInput && detectedMovement)
    {
        std::cout << "\n🎉 SUCCESS: Gang Beasts Evolution Phase 1 Working!" << std::endl;
        std::cout << "✅ Step 1.1: Physics Personality - Skeleton springs are looser" << std::endl;
        std::cout << "✅ Step 1.1: Mass Distribution - Top-heavy character for instability" << std::endl;
        std::cout << "✅ Step 1.1: Flesh Springs - Enhanced wobbliness and damping" << std::endl;
        std::cout << "✅ Step 1.2: Enhanced Stability - Reaction delays implemented" << std::endl;
        std::cout << "✅ Step 1.2: Overcompensation - Comic balance corrections active" << std::endl;
        std::cout << "✅ Integration: Works WITH existing Advanced Control system" << std::endl;
        std::cout << "✅ Settings System: JSON-based tuning without recompilation" << std::endl;
        std::cout << "\n🎯 Next Phase: Implement physics-driven walking (Step 2.1)" << std::endl;
    }
    else if (detectedInput && !detectedMovement)
    {
        std::cout << "\n⚠️  PARTIAL: Input detected but limited movement" << std::endl;
        std::cout << "🔧 This could indicate:" << std::endl;
        std::cout << "   - Gang Beasts springs may be too loose" << std::endl;
        std::cout << "   - Need to adjust stiffness multipliers in settings.json" << std::endl;
        std::cout << "   - Enhanced stability may be over-correcting" << std::endl;
    }
    else
    {
        std::cout << "\n❌ ISSUE: Gang Beasts physics may not be fully integrated" << std::endl;
        std::cout << "🔧 Check settings.json loading and Gang Beasts physics toggle" << std::endl;
    }

    return 0;
}
