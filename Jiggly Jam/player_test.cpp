#include "libs.hpp"
#include "Player.hpp"

// Include implementation directly for a single-translation-unit test build.
// This avoids multiple-definition linker errors because `jelly.hpp` contains
// function implementations and this test compiles everything into one object.
#include "Player.cpp"

int main()
{
    const int W = 1000, H = 700;
    sf::RenderWindow window(sf::VideoMode(W, H), "Player Test - 2 local players");
    window.setFramerateLimit(60);
    // disable OS key repeat to avoid repeated KeyPressed events
    window.setKeyRepeatEnabled(false);

    // create two players
    PlayerControl p1("red");
    Player p2("blue");
    p1.spawn({300.f, 100.f});
    p2.spawn({700.f, 100.f});

    // set ground for both
    float ground_y = H - 80.f;
    p1.set_ground_level(ground_y);
    p2.set_ground_level(ground_y);

    // previous key states for jump edge detection
    bool p1_jump_prev = false;
    bool p2_jump_prev = false;

    // event-driven key state tracking (avoid flaky realtime polling)
    bool p1_left_state = false, p1_right_state = false, p1_jump_state = false;
    bool p2_left_state = false, p2_right_state = false, p2_jump_state = false;

    // fixed timestep
    const float dt = 1.f / 120.f;
    float accumulator = 0.f;
    sf::Clock clock;

    // simple ground visual
    sf::RectangleShape ground(sf::Vector2f((float)W, 80.f));
    ground.setPosition(0.f, H - 80.f);
    ground.setFillColor(sf::Color(100, 180, 100));

    // instructions printed once
    bool shown = false;

    // debug: remember last mapped move to print changes
    MOVE_TYPE last_m1 = MOVE_TYPE::DOWN;
    MOVE_TYPE last_m2 = MOVE_TYPE::DOWN;

    while (window.isOpen())
    {
        sf::Event ev;
        while (window.pollEvent(ev))
        {
            if (ev.type == sf::Event::Closed)
                window.close();
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Escape)
                window.close();

            // update key states on press/release so mapping is stable
            if (ev.type == sf::Event::KeyPressed || ev.type == sf::Event::KeyReleased)
            {
                bool down = (ev.type == sf::Event::KeyPressed);
                // debug print raw event
                std::cout << "EVENT: " << (down ? "Pressed" : "Released") << " code=" << ev.key.code << "\n";
                switch (ev.key.code)
                {
                // player1
                case sf::Keyboard::A:
                    p1_left_state = down;
                    break;
                case sf::Keyboard::D:
                    p1_right_state = down;
                    break;
                case sf::Keyboard::W:
                    p1_jump_state = down;
                    break;
                case sf::Keyboard::Space:
                    p1_jump_state = down;
                    break;
                case sf::Keyboard::Q:
                    if (down)
                        p1.start_wave();
                    break;
                // player2
                case sf::Keyboard::Left:
                    p2_left_state = down;
                    break;
                case sf::Keyboard::Right:
                    p2_right_state = down;
                    break;
                case sf::Keyboard::Up:
                    p2_jump_state = down;
                    break;
                default:
                    break;
                }
            }

            if (ev.type == sf::Event::LostFocus)
            {
                // clear all key states when window loses focus to avoid stuck/flip behavior
                p1_left_state = p1_right_state = p1_jump_state = false;
                p2_left_state = p2_right_state = p2_jump_state = false;
                std::cout << "EVENT: LostFocus - cleared key states\n";
            }
        }

        float frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        // use event-driven key states
        bool p1_left = p1_left_state;
        bool p1_right = p1_right_state;
        bool p1_jump = p1_jump_state;
        bool p2_left = p2_left_state;
        bool p2_right = p2_right_state;
        bool p2_jump = p2_jump_state;

        // edge detection for jumps
        bool p1_jump_edge = p1_jump && !p1_jump_prev;
        bool p2_jump_edge = p2_jump && !p2_jump_prev;
        p1_jump_prev = p1_jump;
        p2_jump_prev = p2_jump;

        // map to MOVE_TYPE for player 1
        MOVE_TYPE m1 = MOVE_TYPE::DOWN;
        if (p1_left && !p1_right)
            m1 = MOVE_TYPE::LEFT;
        else if (p1_right && !p1_left)
            m1 = MOVE_TYPE::RIGHT;
        // explicit jump request below

        // map to MOVE_TYPE for player 2
        MOVE_TYPE m2 = MOVE_TYPE::DOWN;
        if (p2_left && !p2_right)
            m2 = MOVE_TYPE::LEFT;
        else if (p2_right && !p2_left)
            m2 = MOVE_TYPE::RIGHT;
        // explicit jump request below

        // set inputs
        p1.set_input(m1);
        p2.set_input(m2);

        // debug printing when mapping changes
        if (m1 != last_m1)
        {
            std::cout << "P1 mapped move: " << int(m1) << " (left=" << p1_left << " right=" << p1_right << " jump=" << p1_jump << ")\n";
            last_m1 = m1;
        }
        if (m2 != last_m2)
        {
            std::cout << "P2 mapped move: " << int(m2) << " (left=" << p2_left << " right=" << p2_right << " jump=" << p2_jump << ")\n";
            last_m2 = m2;
        }

        // handle edge-triggered jump requests
        if (p1_jump_edge)
            p1.request_jump();
        if (p2_jump_edge)
            p2.request_jump();

        // physics steps
        while (accumulator >= dt)
        {
            p1.update(dt);
            p2.update(dt);
            accumulator -= dt;
        }

        window.clear(sf::Color(40, 40, 50));
        window.draw(ground);

        // draw players by asking for their figure (this returns a copy; acceptable for a test)
        p1.Figure().draw(window);
        p2.Figure().draw(window);

        if (!shown)
        {
            std::cout << "Player test started:\n";
            std::cout << " - Player1: A/D move, W or Space jump\n";
            std::cout << " - Player2: Left/Right arrows move, Up jump\n";
            std::cout << "Close window or press Esc to quit.\n";
            shown = true;
        }

        window.display();
    }

    return 0;
}
