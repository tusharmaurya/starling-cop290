#include "ObsVector.h"
#include <vector>
#include <stdlib.h>
#include <iostream>

#ifndef BOID_H_
#define BOID_H_

// The Boid Class

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
    ObsVector seek(ObsVector v);
    void run(vector <Boid> v);
    void update();
    void flock(vector <Boid> v);
    void borders();
    float angle(ObsVector v);
};

#endif
