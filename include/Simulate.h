#include <iostream>
#include "Flock.h"
#include "Boid.h"
#include "ObsVector.h"
#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"

#ifndef Simulate_H
#define Simulate_H

// Simulate handles the instantiation of a flock of boids, Simulate input, asks the
// model to compute the next step in the stimulation, and handles all of the
// program's interaction with SFML. 

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
