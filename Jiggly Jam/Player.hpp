#pragma once

#include "libs.hpp"
#include "jelly.hpp"

class Player : public Jelly
{
private:
    sf::Vector2f velocity,acceleration,position;
    float height,weight;
    
public:
    Player(/* args */);
    ~Player();
};

Player::Player(/* args */)
{
}

Player::~Player()
{
}
