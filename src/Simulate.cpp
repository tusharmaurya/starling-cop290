#include <iostream>
#include "Flock.h"
#include "Boid.h"
#include "ObsVector.h"
#include "Simulate.h"
#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"
#include <unistd.h>


// Construct window using SFML
Simulate::Simulate()
{
    this->boidsSize = 7.0;
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    this->window_height = desktop.height/2;
    this->window_width  = desktop.width/2;
    this->window.create(sf::VideoMode(window_width, window_height, desktop.bitsPerPixel), "Simulater", sf::Style::None);
}

void Simulate::Run()
{
    for (int i = 0; i < 50; i++) {
        Boid b(window_width /3, window_height / 3); // Starts all boids in the center of the screen
        sf::CircleShape shape(8, 3);

      
        shape.setPosition(window_width, window_height); 
        shape.setOutlineColor(sf::Color(0,255,0));
        shape.setFillColor(sf::Color::Blue);
       // shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1);
        shape.setRadius(boidsSize);

        // Adding the boid to the flock and adding the shapes to the vector<sf::CircleShape>
        flock.addBoid(b);
        shapes.push_back(shape);
    }
    while (window.isOpen()) {
        HandleInput();
        Render();
    }
}

void Simulate::HandleInput()
{
    sf::Event event;
    while (window.pollEvent(event)) {
      
        if ((event.type == sf::Event::Closed) ||
            (event.type == sf::Event::KeyPressed &&
             event.key.code == sf::Keyboard::Escape) ||
            (event.type == sf::Event::KeyPressed &&
             event.key.code == sf::Keyboard::BackSpace) ||
            (event.type == sf::Event::KeyPressed &&
             event.key.code == sf::Keyboard::X))
             {
            window.close();
        }
    }

    // Check for mouse click, draws and adds boid to flock if so.
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
        // Gets mouse coordinates, sets that as the location of the boid and the shape
        sf::Vector2i mouseCoords = sf::Mouse::getPosition(window);
        Boid b(mouseCoords.x, mouseCoords.y, false);
        sf::CircleShape shape(10,3);

        
        shape.setPosition(mouseCoords.x, mouseCoords.y);
        shape.setOutlineColor(sf::Color(255, 0, 0));
        shape.setFillColor(sf::Color(255, 0, 0));
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1);
        shape.setRadius(boidsSize);

        // Adds newly created boid and shape to their respective data structure
        flock.addBoid(b);
        shapes.push_back(shape);

      
        window.draw(shapes[shapes.size()-1]);
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Y)) {

        sf::Vector2i mouseCoords = sf::Mouse::getPosition(window);
        Boid b(mouseCoords.x, mouseCoords.y, true);
        sf::CircleShape shape(10,10);
      
       
        shape.setPosition(mouseCoords.x, mouseCoords.y);
        shape.setOutlineColor(sf::Color(255, 0, 0));
        shape.setFillColor(sf::Color(255, 0, 0));
        shape.setOutlineColor(sf::Color::White);
        shape.setOutlineThickness(1);
        shape.setRadius(boidsSize);

        // Adds newly created boid and shape to their respective data structure
        flock.addBoid(b);
        shapes.push_back(shape);

      
        window.draw(shapes[shapes.size()-1]);
    }
}

void Simulate::Render()
{
    window.clear();

    // Draws all of the Boids out, and applies functions that are needed to update.
    for (int i = 0; i < shapes.size(); i++) {
        window.draw(shapes[i]);

    
        shapes[i].setPosition(flock.getBoid(i).location.x, flock.getBoid(i).location.y);

        // Calculates the angle where the velocity is pointing so that the triangle turns towards it.
        float theta = flock.getBoid(i).angle(flock.getBoid(i).velocity);
        shapes[i].setRotation(theta);

        // Prevent boids from moving off the screen through wrapping
        // If boid exits right boundary
        if (shapes[i].getPosition().x > window_width)
            shapes[i].setPosition(shapes[i].getPosition().x - window_width, shapes[i].getPosition().y);
        // If boid exits bottom boundary
        if (shapes[i].getPosition().y > window_height)
            shapes[i].setPosition(shapes[i].getPosition().x, shapes[i].getPosition().y - window_height);
        // If boid exits left boundary
        if (shapes[i].getPosition().x < 0)
            shapes[i].setPosition(shapes[i].getPosition().x + window_width, shapes[i].getPosition().y);
        // If boid exits top boundary
        if (shapes[i].getPosition().y < 0)
            shapes[i].setPosition(shapes[i].getPosition().x, shapes[i].getPosition().y + window_height);
    }

    // Applies the three rules to each boid in the flock and changes them accordingly.
    flock.flocking();

    window.display();
}
