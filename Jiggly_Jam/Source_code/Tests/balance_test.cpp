#include <iostream>
#include <vector>
#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"

int main()
{
    std::cout << "=== BALANCE FIX TEST ===" << std::endl;

    Player player("red"); // Using string constructor

    player.set_ground_level(400.0f); // Ground at Y=400

    player.spawn({200.0f, 300.0f});

    std::cout << "Player created with " << player.getPointCount() << " points" << std::endl;
    std::cout << "Player ID: " << player.get_id() << std::endl;

    // Test a few physics updates
    std::cout << "\nTesting physics updates..." << std::endl;
    for (int i = 0; i < 10; i++)
    {
        player.update(0.016f); // 60fps

        if (i % 3 == 0)
        {
            sf::Vector2f pos = player.get_position();
            std::cout << "Frame " << i << " - Position: (" << pos.x << ", " << pos.y << ")" << std::endl;
        }
    }

    std::cout << "Balance test completed!" << std::endl;
    return 0;
}
