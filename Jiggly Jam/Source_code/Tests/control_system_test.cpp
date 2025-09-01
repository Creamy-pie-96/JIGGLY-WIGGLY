#include <iostream>
#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"
#include "../../Libraries/Controll_system/BodyControlSystem.hpp"

int main()
{
    std::cout << "🎮 === BODY CONTROL SYSTEM TEST ===" << std::endl;

    // Create window for input system
    sf::RenderWindow window(sf::VideoMode(1200, 800), "Body Control Test");
    window.setFramerateLimit(60);

    // Create player with advanced controls
    Player player("blue", &window);
    player.set_ground_level(700.0f);
    player.spawn({600.0f, 500.0f});

    std::cout << "Player created with " << player.getPointCount() << " points" << std::endl;
    std::cout << "Control System: " << (player.isUsingAdvancedControls() ? "ADVANCED" : "SIMPLE") << std::endl;

    if (player.isUsingAdvancedControls())
    {
        auto *controlSys = player.getControlSystem();
        auto *currentScheme = controlSys->getCurrentScheme();
        std::cout << "Active Scheme: " << currentScheme->getSchemeName() << std::endl;
        std::cout << "Description: " << currentScheme->getControlDescription() << std::endl;
    }

    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 1.0f / 30.0f); // Cap at 30fps minimum

        // Update player (handles control system internally)
        player.update(dt);

        // Render
        window.clear(sf::Color::Black);

        // Draw ground
        sf::RectangleShape ground({1200.0f, 100.0f});
        ground.setPosition(0.0f, 700.0f);
        ground.setFillColor(sf::Color(100, 50, 0));
        window.draw(ground);

        // Draw player
        player.Figure().draw(window);

        // Draw control system debug info
        player.drawControlDebug(window);

        // Instructions
        sf::Text instructions;
        // Note: In a real game, you'd set up font rendering here

        window.display();
    }

    std::cout << "🎮 Body Control System Test Complete!" << std::endl;
    return 0;
}
