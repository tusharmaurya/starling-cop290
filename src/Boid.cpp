#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include "SFML/Graphics.hpp"
#include "Boid.h"


sf::VideoMode desktopTemp = sf::VideoMode::getDesktopMode();
const int window_height = desktopTemp.height/1.5;
const int window_width = desktopTemp.width/1.5;

#define w_height window_height
#define w_width window_width
#define PI 3.141592635


Boid::Boid(float x, float y)
{
    acceleration = ObsVector(0, 0);
    velocity = ObsVector(rand()%3 - 2, rand()%3 - 2);
    location = ObsVector(x, y);
    maxSpeed = 2.5;
    maxForce = 0.5;
}

Boid::Boid(float x, float y, bool Obscheck)
{
    Obstacle = Obscheck;
    if (Obscheck == true) {
        maxSpeed = 0;
        maxForce = 0;
        velocity = ObsVector(0,0);
    } else {
        maxSpeed = 2.5;
        maxForce = 0.5;
        velocity = ObsVector(rand()%3 - 2, rand()%3 - 2);
    }
    acceleration = ObsVector(0, 0);
    location = ObsVector(x, y);
}

// Adds force ObsVector to current force ObsVector
void Boid::applyForce(ObsVector force)
{
    acceleration.addVector(force);
}

// Separation
// Keeps boids from getting too close to one another
ObsVector Boid::Separation(vector<Boid> boids)
{
    // Distance of field of vision for separation between boids
    float desiredseparation = 20;
    ObsVector steer(0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (int i = 0; i < boids.size(); i++) {
        // Calculate distance from current boid to boid we're looking at
        float d = location.distance(boids[i].location);
        // If this is a fellow boid and it's too close, move away from it
        if ((d > 0) && (d < desiredseparation)) {
            ObsVector diff(0,0);
            diff = diff.subTwoVector(location, boids[i].location);
            diff.normalize();
            diff.divScalar(d);      // Weight by distance
            steer.addVector(diff);
            count++;
        }

        // If current boid is not a Obstacle, but the boid we're looking at is
        // a Obstacle, then create a large separation ObsVector
        else if ((d > 0) && (d < desiredseparation+5) && boids[i].Obstacle == true) {
            ObsVector pred(0, 0);
            pred = pred.subTwoVector(location, boids[i].location);
            steer.addVector(pred);
            count++;
        }
    }
    // Adds average difference of location to acceleration
    if (count > 0)
        steer.divScalar((float)count);
    if (steer.magnitude() > 0) {
        
        steer.normalize();
        steer.mulScalar(maxSpeed);
        steer.subVector(velocity);
        steer.limit(maxForce);
    }
    return steer;
}

// Alignment

ObsVector Boid::Alignment(vector<Boid> Boids)
{
    float neighbordist = 50; // Field of vision

    ObsVector sum(0, 0);
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        float d = location.distance(Boids[i].location);
        if ((d > 0) && (d < neighbordist)) { // 0 < d < 50
            sum.addVector(Boids[i].velocity);
            count++;
        }
    }
    // If there are boids close enough for alignment...
    if (count > 0) {
        sum.divScalar((float)count);// Divide sum by the number of close boids (average of velocity)
        sum.normalize();            // Turn sum into a unit vector, and
        sum.mulScalar(maxSpeed);    // Multiply by maxSpeed
        // Steer = Desired - Velocity
        ObsVector steer;
        steer = steer.subTwoVector(sum, velocity); //sum = desired(average)
        steer.limit(maxForce);
        return steer;
    } else {
        ObsVector temp(0, 0);
        return temp;
    }
}

// Cohesion

ObsVector Boid::Cohesion(vector<Boid> Boids)
{
    float neighbordist = 50;
    ObsVector sum(0, 0);
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        float d = location.distance(Boids[i].location);
        if ((d > 0) && (d < neighbordist)) {
            sum.addVector(Boids[i].location);
            count++;
        }
    }
    if (count > 0) {
        sum.divScalar(count);
        return seek(sum);
    } else {
        ObsVector temp(0,0);
        return temp;
    }
}

// Limits the maxSpeed, finds necessary steering force and
// normalizes vectors
ObsVector Boid::seek(ObsVector v)
{
    ObsVector desired;
    desired.subVector(v);  // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mulScalar(maxSpeed);
    // Steering = Desired minus Velocity
    acceleration.subTwoVector(desired, velocity);
    acceleration.limit(maxForce);  // Limit to maximum steering force
    return acceleration;
}

void Boid::update()
{
    //To make the slow down not as abrupt
    acceleration.mulScalar(.4);
    // Update velocity
    velocity.addVector(acceleration);
    // Limit speed
    velocity.limit(maxSpeed);
    location.addVector(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mulScalar(0);
}


void Boid::run(vector <Boid> v)
{
    flock(v);
    update();
    borders();
}

void Boid::flock(vector<Boid> v)
{
    ObsVector sep = Separation(v);
    ObsVector ali = Alignment(v);
    ObsVector coh = Cohesion(v);
    // Arbitrarily weight these forces
    sep.mulScalar(1.5);
    ali.mulScalar(1.0); 
    coh.mulScalar(1.0);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
}

// Checks if boids go out of the window
void Boid::borders()
{
    if (location.x < 0)    location.x += w_width;
    if (location.y < 0)    location.y += w_height;
    if (location.x > w_width) location.x -= w_width;
    if (location.y > w_height) location.y -= w_height;
}

float Boid::angle(ObsVector v)
{
    // From the definition of the dot product
    float angle = (float)(atan2(v.x, -v.y) * 180 / PI);
    return angle;
}
