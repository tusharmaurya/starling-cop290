#include <iostream>
#include "Flock.h"
#include "Boid.h"
#include "ObsVector.h"
#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"

#ifndef Simulate_H
#define Simulate_H



class Simulate {
private:
    sf::RenderWindow window;
    int window_width;
    int window_height;

    Flock flock;
    float boidsSize;
    vector<sf::CircleShape> shapes;

    void Render();
    void HandleInput();

public:
    Simulate();
    void Run();
};

#endif
