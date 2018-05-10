#include "ObsVector.h"
#include <vector>
#include <stdlib.h>
#include <iostream>

#ifndef BOID_H_
#define BOID_H_

// The Boid Class
//
// Attributes
//  bool predator: flag that specifies whether a given boid is a predator.
//  ObsVector location: Vector that specifies a boid's location.
//  ObsVector velocity: Vector that specifies a boid's current velocity.
//  ObsVector acceleration: Vector that specifies a boid's current acceleration.
//  float maxSpeed: Limits magnitude of velocity vector.
//  float maxForce: Limits magnitude of acceleration vector. (F = m*a!)
//
// Methods
//  applyForce(ObsVector force): Adds the given vector to acceleration
//
//  ObsVector Separation(vector<Boid> Boids): If any other boids are within a
//      given distance, Separation computes a a vector that distances the
//      current boid from the boids that are too close.
//
//  ObsVector Alignment(vector<Boid> Boids): Computes a vector that causes the
//      velocity of the current boid to match that of boids that are nearby.
//
//  ObsVector Cohesion(vector<Boid> Boids): Computes a vector that causes the
//      current boid to seek the center of mass of nearby boids.

class Boid {
public:
    bool Obstacle;
    ObsVector location;
    ObsVector velocity;
    ObsVector acceleration;
    float maxSpeed;
    float maxForce;
    Boid() {}
    Boid(float x, float y);
    Boid(float x, float y, bool Obscheck);
    void applyForce(ObsVector force);
    // Three Laws that boids follow
    ObsVector Separation(vector<Boid> Boids);
    ObsVector Alignment(vector<Boid> Boids);
    ObsVector Cohesion(vector<Boid> Boids);
    //Functions involving SFML and visualisation linking
    ObsVector seek(ObsVector v);
    void run(vector <Boid> v);
    void update();
    void flock(vector <Boid> v);
    void borders();
    float angle(ObsVector v);
};

#endif
