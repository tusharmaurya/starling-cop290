#include <iostream>

using namespace std;

#ifndef OBSVECTOR_H_
#define OBSVECTOR_H_



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


    void set(float x, float y);

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

  
    float distance(ObsVector v);
    float dotProduct(ObsVector v);
    float magnitude();
    void setMagnitude(float x);
    float angleBetween(ObsVector v);
    void normalize();

    ObsVector copy(ObsVector v);
};

#endif
