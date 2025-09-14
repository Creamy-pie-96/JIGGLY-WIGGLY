#include "../../Libraries/libs.hpp"
#include "../../Libraries/Player/Player.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

std::string bodyPartToString(BODY_PART part)
{
    switch (part)
    {
    case BODY_PART::NONE:
        return "FLESH";
    case BODY_PART::HEAD:
        return "HEAD";
    case BODY_PART::NECK:
        return "NECK";
    case BODY_PART::SPINE_UP:
        return "SPINE_UP";
    case BODY_PART::SPINE_MID:
        return "SPINE_MID";
    case BODY_PART::SPINE_LOW:
        return "SPINE_LOW";
    case BODY_PART::CLAV_R:
        return "CLAV_R";
    case BODY_PART::CLAV_L:
        return "CLAV_L";
    case BODY_PART::SHO_R:
        return "SHO_R";
    case BODY_PART::ELB_R:
        return "ELB_R";
    case BODY_PART::WRIST_R:
        return "WRIST_R";
    case BODY_PART::HAND_R:
        return "HAND_R";
    case BODY_PART::SHO_L:
        return "SHO_L";
    case BODY_PART::ELB_L:
        return "ELB_L";
    case BODY_PART::WRIST_L:
        return "WRIST_L";
    case BODY_PART::HAND_L:
        return "HAND_L";
    case BODY_PART::PELVIS:
        return "PELVIS";
    case BODY_PART::HIP_L:
        return "HIP_L";
    case BODY_PART::KNEE_L:
        return "KNEE_L";
    case BODY_PART::ANKLE_L:
        return "ANKLE_L";
    case BODY_PART::FOOT_L:
        return "FOOT_L";
    case BODY_PART::HIP_R:
        return "HIP_R";
    case BODY_PART::KNEE_R:
        return "KNEE_R";
    case BODY_PART::ANKLE_R:
        return "ANKLE_R";
    case BODY_PART::FOOT_R:
        return "FOOT_R";
    default:
        return "UNKNOWN";
    }
}

int main()
{
    const int W = 1200, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "Debug Springs Test - Skeleton vs Flesh");
    window.setFramerateLimit(60);

    // create a single player for detailed inspection
    PlayerControl player("green");

    bool settingsLoaded = player.loadGangBeastsSettings("Jiggly_Jam/Game Settings/settings.json");
    if (settingsLoaded)
    {
        std::cout << "✅ Gang Beasts settings loaded successfully!" << std::endl;
    }
    else
    {
        std::cout << "⚠️ Gang Beasts settings failed to load - using defaults" << std::endl;
    }

    player.spawn({W / 2.f, 200.f});

    float ground_y = H - 100.f;
    player.set_ground_level(ground_y);

    // fixed timestep
    const float dt = Player::STABLE_TIMESTEP; // Use the stable timestep from Player class!
    float accumulator = 0.f;
    sf::Clock clock;

    // ground visual
    sf::RectangleShape ground(sf::Vector2f((float)W, 100.f));
    ground.setPosition(0.f, H - 100.f);
    ground.setFillColor(sf::Color(80, 120, 80));

    // debug mode toggles
    bool show_skeleton_springs = true;
    bool show_flesh_springs = true;
    bool show_point_numbers = false;
    bool show_point_markers = true;
    bool show_mass_distribution = true;
    bool show_stability_forces = true;
    bool show_com_debug = true;
    bool show_body_part_labels = false;
    bool log_physics_events = true;

    // Body part cycling variables
    int current_body_part_index = -1; // -1 means off, 0+ means showing specific part
    std::vector<BODY_PART> body_parts_list = {
        BODY_PART::HEAD, BODY_PART::NECK, BODY_PART::SPINE_UP, BODY_PART::SPINE_MID, BODY_PART::SPINE_LOW,
        BODY_PART::CLAV_R, BODY_PART::CLAV_L, BODY_PART::SHO_R, BODY_PART::ELB_R, BODY_PART::WRIST_R, BODY_PART::HAND_R,
        BODY_PART::SHO_L, BODY_PART::ELB_L, BODY_PART::WRIST_L, BODY_PART::HAND_L, BODY_PART::PELVIS,
        BODY_PART::HIP_L, BODY_PART::KNEE_L, BODY_PART::ANKLE_L, BODY_PART::FOOT_L,
        BODY_PART::HIP_R, BODY_PART::KNEE_R, BODY_PART::ANKLE_R, BODY_PART::FOOT_R};

    // text for debug info
    sf::Font font;
    font.loadFromFile("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");

    sf::Text debugText;
    debugText.setFont(font);
    debugText.setCharacterSize(16);
    debugText.setFillColor(sf::Color::White);
    debugText.setPosition(10, 10);

    while (window.isOpen())
    {
        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed)
            {
                switch (event.key.code)
                {
                case sf::Keyboard::Escape:
                    window.close();
                    break;
                case sf::Keyboard::Num1:
                    show_skeleton_springs = !show_skeleton_springs;
                    break;
                case sf::Keyboard::Num2:
                    show_flesh_springs = !show_flesh_springs;
                    break;
                case sf::Keyboard::Num3:
                    show_point_numbers = !show_point_numbers;
                    break;
                case sf::Keyboard::Num4:
                    show_point_markers = !show_point_markers;
                    break;
                case sf::Keyboard::Num5:
                    show_mass_distribution = !show_mass_distribution;
                    break;
                case sf::Keyboard::Num6:
                    show_stability_forces = !show_stability_forces;
                    break;
                case sf::Keyboard::Num7:
                    show_com_debug = !show_com_debug;
                    break;
                case sf::Keyboard::Num8:
                    log_physics_events = !log_physics_events;
                    break;
                case sf::Keyboard::Num9:
                    // Cycle through body parts: off -> part 0 -> part 1 -> ... -> part N -> off
                    current_body_part_index++;
                    if (current_body_part_index >= (int)body_parts_list.size())
                    {
                        current_body_part_index = -1; // Turn off after showing all parts
                        show_body_part_labels = false;
                    }
                    else
                    {
                        show_body_part_labels = true;
                    }
                    break;
                case sf::Keyboard::Space:
                    player.request_jump();
                    break;
                case sf::Keyboard::W:
                    player.start_wave(); // test the waving animation
                    break;
                case sf::Keyboard::R:
                    // reset player position
                    player.spawn({W / 2.f, 200.f});
                    break;
                default:
                    break;
                }
            }
        }

        // basic movement input
        MOVE_TYPE move = MOVE_TYPE::DOWN;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) || sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            move = MOVE_TYPE::LEFT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) || sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            move = MOVE_TYPE::RIGHT;

        player.set_input(move);

        // fixed timestep physics
        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        while (accumulator >= dt)
        {
            player.update(dt);
            accumulator -= dt;
        }

        // render
        window.clear(sf::Color(40, 50, 60));

        // draw ground
        window.draw(ground);

        // get figure reference
        Jelly figure = player.Figure(); // copy instead of reference

        // custom spring drawing with color coding
        if (show_skeleton_springs || show_flesh_springs)
        {
            for (const auto &spring : figure.springs)
            {
                if (spring.p1 >= 0 && spring.p1 < (int)figure.points.size() &&
                    spring.p2 >= 0 && spring.p2 < (int)figure.points.size())
                {
                    sf::Vector2f p1 = figure.points[spring.p1].pos;
                    sf::Vector2f p2 = figure.points[spring.p2].pos;

                    sf::Color springColor;
                    float thickness = 1.f;

                    if (spring.is_skeleton)
                    {
                        if (!show_skeleton_springs)
                            continue;
                        springColor = sf::Color::Red; // skeleton springs in red
                        thickness = 2.f;
                    }
                    else if (spring.owner_id == player.get_id())
                    {
                        if (!show_flesh_springs)
                            continue;
                        springColor = sf::Color::Cyan; // flesh-to-skeleton in cyan
                        thickness = 1.f;
                    }
                    else
                    {
                        if (!show_flesh_springs)
                            continue;
                        springColor = sf::Color(100, 100, 100, 150); // regular springs in gray
                        thickness = 0.5f;
                    }

                    sf::Vertex line[] = {
                        sf::Vertex(p1, springColor),
                        sf::Vertex(p2, springColor)};
                    window.draw(line, 2, sf::Lines);
                }
            }
        }

        if (figure.points.size() > 1)
        {
            // draw filled shape from peripheral points - estimate perimeter count
            int peripheralCount = std::min(50, (int)figure.points.size() - 1); // rough estimate
            if (peripheralCount >= 3)
            {
                sf::ConvexShape shape;
                shape.setPointCount(peripheralCount);
                for (int i = 0; i < peripheralCount; ++i)
                {
                    shape.setPoint(i, figure.points[i + 1].pos);
                }
                shape.setFillColor(sf::Color(120, 180, 240, 200));
                window.draw(shape);
            }
        }

        // selectively draw point markers
        if (show_point_markers)
        {
            for (size_t i = 0; i < figure.points.size(); ++i)
            {
                sf::CircleShape c(3.f);
                c.setOrigin(3.f, 3.f);
                c.setPosition(figure.points[i].pos);

                // color code points: skeleton joints in red, flesh in blue
                if (figure.points[i].body_part != BODY_PART::NONE)
                {
                    c.setFillColor(sf::Color::Red); // skeleton joints
                }
                else
                {
                    c.setFillColor(sf::Color::Blue); // flesh points
                }
                window.draw(c);
            }
        }

        // Debug visualization: Mass distribution
        if (show_mass_distribution)
        {
            for (size_t i = 0; i < figure.points.size(); ++i)
            {
                const auto &point = figure.points[i];
                if (point.id == player.get_id())
                {
                    float mass_scale = std::min(point.mass * 2.0f, 15.0f); // Scale and cap
                    sf::CircleShape massCircle(mass_scale);
                    massCircle.setOrigin(mass_scale, mass_scale);
                    massCircle.setPosition(point.pos);

                    // Color based on mass magnitude
                    sf::Uint8 intensity = (sf::Uint8)std::min(255.0f, point.mass * 30.0f);
                    massCircle.setFillColor(sf::Color(intensity, 255 - intensity, 0, 100)); // red=heavy, green=light
                    massCircle.setOutlineThickness(1.0f);
                    massCircle.setOutlineColor(sf::Color::White);

                    window.draw(massCircle);
                }
            }
        }

        // Debug visualization: Center of Mass and Base of Support
        auto physicsDebugInfo = figure.getDebugInfo(player.get_id());

        if (show_com_debug)
        {
            sf::CircleShape comMarker(8.0f);
            comMarker.setOrigin(8.0f, 8.0f);
            comMarker.setPosition(physicsDebugInfo.center_of_mass);
            comMarker.setFillColor(sf::Color::Magenta);
            comMarker.setOutlineThickness(2.0f);
            comMarker.setOutlineColor(sf::Color::White);
            window.draw(comMarker);

            sf::CircleShape bosMarker(6.0f);
            bosMarker.setOrigin(6.0f, 6.0f);
            bosMarker.setPosition(physicsDebugInfo.base_of_support);
            bosMarker.setFillColor(sf::Color::Green);
            bosMarker.setOutlineThickness(2.0f);
            bosMarker.setOutlineColor(sf::Color::White);
            window.draw(bosMarker);

            sf::Vertex comLine[] = {
                sf::Vertex(physicsDebugInfo.center_of_mass, sf::Color::Yellow),
                sf::Vertex(physicsDebugInfo.base_of_support, sf::Color::Yellow)};
            window.draw(comLine, 2, sf::Lines);

            // DEBUG: Print foot positions to check if they're the same
            static int debug_counter = 0;
            if (++debug_counter % 120 == 0) // Every 2 seconds
            {
                std::cout << "DEBUG FOOT POSITIONS:" << std::endl;
                for (size_t i = 0; i < physicsDebugInfo.foot_positions.size(); ++i)
                {
                    std::cout << "  Foot " << i << " (";
                    std::cout << (i == 0 ? "LEFT" : "RIGHT") << "): ";
                    std::cout << "(" << physicsDebugInfo.foot_positions[i].x << ", " << physicsDebugInfo.foot_positions[i].y << ")";
                    std::cout << " Grounded: " << ((i == 0) ? physicsDebugInfo.left_foot_grounded : physicsDebugInfo.right_foot_grounded);
                    std::cout << " Supporting: " << ((i == 0) ? physicsDebugInfo.support_legs[1] : physicsDebugInfo.support_legs[0]);
                    std::cout << std::endl;
                }
                std::cout << std::endl;
            }

            for (size_t i = 0; i < physicsDebugInfo.foot_positions.size(); ++i)
            {
                sf::CircleShape footMarker(3.0f); // Smaller radius to reduce overlap
                footMarker.setOrigin(3.0f, 3.0f);

                sf::Vector2f footPos = physicsDebugInfo.foot_positions[i];
                if (i == 0)            // Left foot
                    footPos.x -= 2.0f; // Offset left foot slightly to the left
                else                   // Right foot
                    footPos.x += 2.0f; // Offset right foot slightly to the right

                footMarker.setPosition(footPos);

                bool grounded = (i == 0) ? physicsDebugInfo.left_foot_grounded : physicsDebugInfo.right_foot_grounded;
                bool supporting = (i == 0) ? physicsDebugInfo.support_legs[1] : physicsDebugInfo.support_legs[0]; // left foot = index 1, right foot = index 0

                // Color: Green if grounded, Red if not grounded
                footMarker.setFillColor(grounded ? sf::Color::Green : sf::Color::Red);

                // Outline: Thick yellow if supporting, thin white if not supporting
                if (supporting)
                {
                    footMarker.setOutlineThickness(2.0f); // Slightly thinner outline
                    footMarker.setOutlineColor(sf::Color::Yellow);
                }
                else
                {
                    footMarker.setOutlineThickness(1.0f);
                    footMarker.setOutlineColor(sf::Color::White);
                }

                window.draw(footMarker);
            }
        }

        // Log physics events if enabled
        if (log_physics_events)
        {
            static int log_counter = 0;
            if (++log_counter % 60 == 0) // Every 60 frames (1 second at 60fps)
            {
                sf::Vector2f com_to_bos = physicsDebugInfo.center_of_mass - physicsDebugInfo.base_of_support;
                float distance = std::sqrt(com_to_bos.x * com_to_bos.x + com_to_bos.y * com_to_bos.y);

                std::string support_status = "";
                if (physicsDebugInfo.support_legs[0] && physicsDebugInfo.support_legs[1])
                {
                    support_status = "BOTH";
                }
                else if (physicsDebugInfo.support_legs[0])
                {
                    support_status = "RIGHT";
                }
                else if (physicsDebugInfo.support_legs[1])
                {
                    support_status = "LEFT";
                }
                else
                {
                    support_status = "NONE";
                }

                std::cout << "COM-BOS Distance: " << distance
                          << " | Imbalance: " << physicsDebugInfo.imbalance_magnitude
                          << " | L_Foot: " << (physicsDebugInfo.left_foot_grounded ? "GND" : "AIR")
                          << " | R_Foot: " << (physicsDebugInfo.right_foot_grounded ? "GND" : "AIR")
                          << " | Support: " << support_status
                          << " | Walking: " << (physicsDebugInfo.is_walking ? "YES" : "NO")
                          << " | Grounded: " << (physicsDebugInfo.is_grounded ? "YES" : "NO")
                          << " | Phase: " << std::fixed << std::setprecision(2) << physicsDebugInfo.step_phase
                          << std::endl;
            }
        }

        // optional: draw point numbers for debugging
        if (show_point_numbers)
        {
            for (size_t i = 0; i < figure.points.size(); ++i)
            {
                sf::Text pointText;
                pointText.setFont(font);
                pointText.setString(std::to_string(i));
                pointText.setCharacterSize(12);
                pointText.setFillColor(sf::Color::Yellow);
                pointText.setPosition(figure.points[i].pos.x + 5, figure.points[i].pos.y - 5);
                window.draw(pointText);
            }
        }

        if (show_body_part_labels && current_body_part_index >= 0 && current_body_part_index < (int)body_parts_list.size())
        {
            BODY_PART target_part = body_parts_list[current_body_part_index];

            // DEBUG: Print all points with this body part type for debugging
            static int debug_counter = 0;
            if (++debug_counter % 60 == 0) // Every 1 second
            {
                std::cout << "DEBUG: Looking for " << bodyPartToString(target_part) << " points:" << std::endl;
                int count = 0;
                for (size_t i = 0; i < figure.points.size(); ++i)
                {
                    if (figure.points[i].id == player.get_id() && figure.points[i].body_part == target_part)
                    {
                        std::cout << "  Point " << i << ": (" << figure.points[i].pos.x << ", " << figure.points[i].pos.y << ")" << std::endl;
                        count++;
                    }
                }
                std::cout << "  Total points found: " << count << std::endl;
                std::cout << std::endl;
            }

            for (size_t i = 0; i < figure.points.size(); ++i)
            {
                if (figure.points[i].id == player.get_id() && figure.points[i].body_part == target_part)
                {
                    sf::CircleShape bodyPartMarker(8.0f); // Slightly larger since it's the only one
                    bodyPartMarker.setOrigin(8.0f, 8.0f);
                    bodyPartMarker.setPosition(figure.points[i].pos);
                    bodyPartMarker.setFillColor(sf::Color::Cyan);
                    bodyPartMarker.setOutlineThickness(3.0f);
                    bodyPartMarker.setOutlineColor(sf::Color::White);
                    window.draw(bodyPartMarker);

                    sf::Text bodyPartText;
                    bodyPartText.setFont(font);
                    bodyPartText.setString(bodyPartToString(figure.points[i].body_part));
                    bodyPartText.setCharacterSize(14); // Larger text since it's the only one
                    bodyPartText.setFillColor(sf::Color::Cyan);
                    bodyPartText.setStyle(sf::Text::Bold);

                    sf::Vector2f labelPos = figure.points[i].pos;
                    labelPos.x += 12;
                    labelPos.y -= 12;
                    bodyPartText.setPosition(labelPos);
                    window.draw(bodyPartText);

                    // Show the exact coordinates for this part
                    sf::Text coordText;
                    coordText.setFont(font);
                    coordText.setString("(" + std::to_string((int)figure.points[i].pos.x) + ", " + std::to_string((int)figure.points[i].pos.y) + ")");
                    coordText.setCharacterSize(10);
                    coordText.setFillColor(sf::Color::Yellow);
                    sf::Vector2f coordPos = figure.points[i].pos;
                    coordPos.x += 12;
                    coordPos.y += 5;
                    coordText.setPosition(coordPos);
                    window.draw(coordText);

                    break; // Only show one instance of this body part
                }
            }

            // Always draw COM and BOS labels when showing body parts
            auto physicsDebugInfo = figure.getDebugInfo(player.get_id());
            sf::Text comText;
            comText.setFont(font);
            comText.setString("COM");
            comText.setCharacterSize(12);
            comText.setFillColor(sf::Color::Red);
            comText.setStyle(sf::Text::Bold);
            sf::Vector2f comLabelPos = physicsDebugInfo.center_of_mass;
            comLabelPos.x += 10;
            comLabelPos.y -= 10;
            comText.setPosition(comLabelPos);
            window.draw(comText);

            sf::Text bosText;
            bosText.setFont(font);
            bosText.setString("BOS");
            bosText.setCharacterSize(12);
            bosText.setFillColor(sf::Color::Green);
            bosText.setStyle(sf::Text::Bold);
            sf::Vector2f bosLabelPos = physicsDebugInfo.base_of_support;
            bosLabelPos.x += 10;
            bosLabelPos.y += 15;
            bosText.setPosition(bosLabelPos);
            window.draw(bosText);
        }

        // debug info
        std::string debugInfo =
            std::string("SPRINGS DEBUG:\n") +
            "1: Toggle skeleton springs (red) [" + (show_skeleton_springs ? "ON" : "OFF") + "]\n" +
            "2: Toggle flesh springs (cyan) [" + (show_flesh_springs ? "ON" : "OFF") + "]\n" +
            "3: Toggle point numbers [" + (show_point_numbers ? "ON" : "OFF") + "]\n" +
            "4: Toggle point markers [" + (show_point_markers ? "ON" : "OFF") + "]\n" +
            "5: Toggle mass distribution [" + (show_mass_distribution ? "ON" : "OFF") + "]\n" +
            "6: Toggle stability forces [" + (show_stability_forces ? "ON" : "OFF") + "]\n" +
            "7: Toggle COM/BOS debug [" + (show_com_debug ? "ON" : "OFF") + "]\n" +
            "8: Toggle physics event logs [" + (log_physics_events ? "ON" : "OFF") + "]\n" +
            "9: Toggle body part labels [" + (show_body_part_labels ? (current_body_part_index >= 0 && current_body_part_index < (int)body_parts_list.size() ? bodyPartToString(body_parts_list[current_body_part_index]) : "ON") : "OFF") + "]\n" +
            "WASD/Arrows: Move\n" +
            "Space: Jump\n" +
            "W: Wave animation\n" +
            "R: Reset position\n" +
            "Esc: Exit\n\n" +
            "Points: " + std::to_string(figure.points.size()) + "\n" +
            "Springs: " + std::to_string(figure.springs.size()) + "\n" +
            "Player ID: " + std::to_string(player.get_id()) + "\n\n" +
            "COM: (" + std::to_string((int)physicsDebugInfo.center_of_mass.x) + ", " + std::to_string((int)physicsDebugInfo.center_of_mass.y) + ")\n" +
            "BOS: (" + std::to_string((int)physicsDebugInfo.base_of_support.x) + ", " + std::to_string((int)physicsDebugInfo.base_of_support.y) + ")\n" +
            "Imbalance: " + std::to_string(physicsDebugInfo.imbalance_magnitude) + "\n" +
            "L_Foot: " + (physicsDebugInfo.left_foot_grounded ? "GROUNDED" : "AIRBORNE") + "\n" +
            "R_Foot: " + (physicsDebugInfo.right_foot_grounded ? "GROUNDED" : "AIRBORNE");

        debugText.setString(debugInfo);
        window.draw(debugText);

        window.display();
    }

    return 0;
}
