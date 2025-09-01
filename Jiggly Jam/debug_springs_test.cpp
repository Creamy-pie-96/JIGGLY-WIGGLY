#include "libs.hpp"
#include "Player.hpp"
#include "Player.cpp"

int main()
{
    const int W = 1200, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "Debug Springs Test - Skeleton vs Flesh");
    window.setFramerateLimit(60);

    // create a single player for detailed inspection
    PlayerControl player("green");
    player.spawn({W / 2.f, 200.f});

    float ground_y = H - 100.f;
    player.set_ground_level(ground_y);

    // fixed timestep
    const float dt = 1.f / 120.f;
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

    // text for debug info
    sf::Font font;
    // try to load a system font (may fail but won't crash)
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

                    // draw line (simple approximation)
                    sf::Vertex line[] = {
                        sf::Vertex(p1, springColor),
                        sf::Vertex(p2, springColor)};
                    window.draw(line, 2, sf::Lines);
                }
            }
        }

        // draw the figure shape only (without automatic point drawing)
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

        // debug info
        std::string debugInfo =
            std::string("SPRINGS DEBUG:\n") +
            "1: Toggle skeleton springs (red) [" + (show_skeleton_springs ? "ON" : "OFF") + "]\n" +
            "2: Toggle flesh springs (cyan) [" + (show_flesh_springs ? "ON" : "OFF") + "]\n" +
            "3: Toggle point numbers [" + (show_point_numbers ? "ON" : "OFF") + "]\n" +
            "4: Toggle point markers [" + (show_point_markers ? "ON" : "OFF") + "]\n" +
            "WASD/Arrows: Move\n" +
            "Space: Jump\n" +
            "W: Wave animation\n" +
            "R: Reset position\n" +
            "Esc: Exit\n\n" +
            "Points: " + std::to_string(figure.points.size()) + "\n" +
            "Springs: " + std::to_string(figure.springs.size()) + "\n" +
            "Player ID: " + std::to_string(player.get_id());

        debugText.setString(debugInfo);
        window.draw(debugText);

        window.display();
    }

    return 0;
}
