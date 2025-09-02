#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

class Human
{
private:
    float s;
    sf::Vector2f origin;
    float height;

    // Body parts as separate shapes
    sf::RectangleShape torso;
    sf::RectangleShape neck;
    sf::CircleShape head;
    sf::RectangleShape rightArm;
    sf::RectangleShape leftArm;
    sf::RectangleShape rightHand;
    sf::RectangleShape leftHand;
    sf::RectangleShape rightLeg;
    sf::RectangleShape leftLeg;
    sf::RectangleShape rightFoot;
    sf::RectangleShape leftFoot;

public:
    Human(const sf::Vector2f &orig, const float &h) : origin(orig), height(h)
    {
        s = h / 175.f; // Scale factor based on height

        // Define dimensions for each body part relative to the scale
        float headRadius = 15.f * s;
        float limbWidth = 8.f * s;
        float torsoWidth = 25.f * s;
        float torsoHeight = 50.f * s;
        float limbLength = 50.f * s;
        float handFootSize = 10.f * s;

        // Head and Neck
        head.setRadius(headRadius);
        head.setPosition(origin.x - headRadius, origin.y - torsoHeight / 2 - headRadius * 2);
        
        neck.setSize(sf::Vector2f(limbWidth, 10.f * s));
        neck.setPosition(head.getPosition().x + headRadius - limbWidth / 2, head.getPosition().y + headRadius * 2);

        // Torso
        torso.setSize(sf::Vector2f(torsoWidth, torsoHeight));
        torso.setPosition(neck.getPosition().x + limbWidth / 2 - torsoWidth / 2, neck.getPosition().y + neck.getSize().y);

        // Arms (slightly rotated outward to form a 'V' shape)
        rightArm.setSize(sf::Vector2f(limbWidth, limbLength));
        rightArm.setOrigin(limbWidth / 2, 0);
        rightArm.setPosition(torso.getPosition().x + torsoWidth, torso.getPosition().y);
        rightArm.setRotation(-15); // Rotated slightly outward

        leftArm.setSize(sf::Vector2f(limbWidth, limbLength));
        leftArm.setOrigin(limbWidth / 2, 0);
        leftArm.setPosition(torso.getPosition().x, torso.getPosition().y);
        leftArm.setRotation(15); // Rotated slightly outward

        // Hands
        rightHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        rightHand.setOrigin(handFootSize / 2, 0);
        // Position hands relative to the end of the rotated arms
        rightHand.setPosition(rightArm.getGlobalBounds().left + rightArm.getGlobalBounds().width / 2, rightArm.getGlobalBounds().top + rightArm.getGlobalBounds().height);

        leftHand.setSize(sf::Vector2f(handFootSize, handFootSize));
        leftHand.setOrigin(handFootSize / 2, 0);
        // Position hands relative to the end of the rotated arms
        leftHand.setPosition(leftArm.getGlobalBounds().left + leftArm.getGlobalBounds().width / 2, leftArm.getGlobalBounds().top + leftArm.getGlobalBounds().height);

        // Legs (straight)
        rightLeg.setSize(sf::Vector2f(limbWidth, limbLength));
        rightLeg.setOrigin(limbWidth / 2, 0);
        rightLeg.setPosition(torso.getPosition().x + torsoWidth, torso.getPosition().y + torsoHeight);
        rightLeg.setRotation(0);

        leftLeg.setSize(sf::Vector2f(limbWidth, limbLength));
        leftLeg.setOrigin(limbWidth / 2, 0);
        leftLeg.setPosition(torso.getPosition().x, torso.getPosition().y + torsoHeight);
        leftLeg.setRotation(0);

        // Feet
        rightFoot.setSize(sf::Vector2f(handFootSize * 1.5f, handFootSize * 0.75f));
        rightFoot.setOrigin(handFootSize * 0.75f, 0);
        rightFoot.setPosition(rightLeg.getPosition().x, rightLeg.getPosition().y + limbLength);

        leftFoot.setSize(sf::Vector2f(handFootSize * 1.5f, handFootSize * 0.75f));
        leftFoot.setOrigin(handFootSize * 0.75f, 0);
        leftFoot.setPosition(leftLeg.getPosition().x, leftLeg.getPosition().y + limbLength);

        // Set all shapes to a red fill color
        head.setFillColor(sf::Color::Red);
        neck.setFillColor(sf::Color::Red);
        torso.setFillColor(sf::Color::Red);
        rightArm.setFillColor(sf::Color::Red);
        leftArm.setFillColor(sf::Color::Red);
        rightLeg.setFillColor(sf::Color::Red);
        leftLeg.setFillColor(sf::Color::Red);
        rightHand.setFillColor(sf::Color::Red);
        leftHand.setFillColor(sf::Color::Red);
        rightFoot.setFillColor(sf::Color::Red);
        leftFoot.setFillColor(sf::Color::Red);
    }
    ~Human() = default;

    void draw(sf::RenderWindow& window)
    {
        window.draw(head);
        window.draw(neck);
        window.draw(torso);
        window.draw(rightArm);
        window.draw(leftArm);
        window.draw(rightLeg);
        window.draw(leftLeg);
        window.draw(rightHand);
        window.draw(leftHand);
        window.draw(rightFoot);
        window.draw(leftFoot);
    }
};

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Human Figure");
    window.setFramerateLimit(60);

    Human myHuman(sf::Vector2f(400.f, 300.f), 175.f);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color(50, 50, 50));
        myHuman.draw(window);
        window.display();
    }

    return 0;
}
