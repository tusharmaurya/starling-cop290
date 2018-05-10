#include <iostream>

using namespace std;

#ifndef OBSVECTOR_H_
#define OBSVECTOR_H_

// The ObsVector class implements Euclidian vectors -- that is, each vector has
// both a magnitude and a direction. We use ObsVectors for implementing movement
// and the three Boid rules -- cohesion, separation, and matching velocity
// through the use of acceleration, force, and velocity vectors.

class ObsVector {

public:
    float x;
    float y;

    //Constructors
    ObsVector() {}

    ObsVector(float xComp, float yComp)
    {
        x = xComp;
        y = yComp;
    }

    //Mutator Functions
    void set(float x, float y);

    //Scalar functions scale a vector by a float
    void addVector(ObsVector v);
    void addScalar(float x);

    void subVector(ObsVector v);
    ObsVector subTwoVector(ObsVector v, ObsVector v2);
    void subScalar(float x);

    void mulVector(ObsVector v);
    void mulScalar(float x);

    void divVector(ObsVector v);
    void divScalar(float x);

    void limit(double max);

    //Calculating Functions
    float distance(ObsVector v);
    float dotProduct(ObsVector v);
    float magnitude();
    void setMagnitude(float x);
    float angleBetween(ObsVector v);
    void normalize();

    ObsVector copy(ObsVector v);
};

#endif
