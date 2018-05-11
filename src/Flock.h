#include <iostream>
#include <vector>
#include "Boid.h"

#ifndef FLOCK_H_
#define FLOCK_H_



class Flock {
public:
    vector<Boid> flock;
    Flock() {}
    int getSize();
    Boid getBoid(int i);
    void addBoid(Boid b);
    void flocking();
};

#endif
