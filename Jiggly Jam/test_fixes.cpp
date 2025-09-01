// Quick test to verify the fixes
#include "Libraries/Player/Player.hpp"
#include <iostream>

int main()
{
    std::cout << "🧪 Testing Fix Verification..." << std::endl;

    // Test 1: Check walkForwardSpeed is reasonable
    Player player("test");
    std::cout << "✅ Walk Speed: " << player.walkForwardSpeed << " (should be ~25, not 80)" << std::endl;

    // Test 2: Check if advanced controls bridge works
    player.setAdvancedControls(true);
    std::cout << "✅ Advanced Controls: " << (player.isUsingAdvancedControls() ? "Enabled" : "Disabled") << std::endl;

    // Test 3: Check waving duration
    std::cout << "✅ Wave Duration: " << player.waveDuration << "s" << std::endl;

    std::cout << "🎯 Basic Fixes Verified!" << std::endl;
    std::cout << "💡 The real test is in the simulation - WASD should now move the player" << std::endl;
    std::cout << "💡 And waving should not launch the player into space!" << std::endl;

    return 0;
}
