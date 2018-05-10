#include <math.h>
#include "ObsVector.h"

// =================================================== //
// ======== ObsVector Functions from ObsVector.h ========= //
// =================================================== //

#define PI 3.141592635

// Sets values of x and y for ObsVector
void ObsVector::set(float i, float o)
{
    x = i;
    y = o;
}

void ObsVector::addVector(ObsVector v)
{
    x += v.x;
    y += v.y;
}

// Adds to a ObsVector by a constant number
void ObsVector::addScalar(float s)
{
    x += s;
    y += s;
}

// Subtracts 2 vectors
void ObsVector::subVector(ObsVector v)
{
    x -= v.x;
    y -= v.y;
}

// Subtracts two vectors and returns the difference as a vector
ObsVector ObsVector::subTwoVector(ObsVector v, ObsVector v2)
{
    ObsVector tmp;
    v.x -= v2.x;
    v.y -= v2.y;
    tmp.set(v.x, v.y);
    return tmp;
}

// Adds to a ObsVector by a constant number
void ObsVector::subScalar(float s)
{
    x -= s;
    y -= s;
}

// Multiplies 2 vectors
void ObsVector::mulVector(ObsVector v)
{
    x *= v.x;
    y *= v.y;
}

// Adds to a ObsVector by a constant number
void ObsVector::mulScalar(float s)
{
    x *= s;
    y *= s;
}

// Divides 2 vectors
void ObsVector::divVector(ObsVector v)
{
    x /= v.x;
    y /= v.y;
}

// Adds to a ObsVector by a constant number
void ObsVector::divScalar(float s)
{
    x /= s;
    y /= s;
}

void ObsVector::limit(double max)
{
    double size = magnitude();

    if (size > max) {
        set(x / size, y / size);
    }
}

// Calculates the distance between the first ObsVector and second ObsVector
float ObsVector::distance(ObsVector v)
{
    float dx = x - v.x;
    float dy = y - v.y;
    float dist = sqrt(dx*dx + dy*dy);
    return dist;
}

// Calculates the dot product of a vector
float ObsVector::dotProduct(ObsVector v)
{
    float dot = x * v.x + y * v.y;
    return dot;
}

// Calculates magnitude of referenced object
float ObsVector::magnitude()
{
    return sqrt(x*x + y*y);
}

void ObsVector::setMagnitude(float x)
{
    normalize();
    mulScalar(x);
}

// Calculate the angle between ObsVector 1 and ObsVector 2
float ObsVector::angleBetween(ObsVector v)
{
    if (x == 0 && y == 0) return 0.0f;
    if (v.x == 0 && v.y == 0) return 0.0f;

    double dot = x * v.x + y * v.y;
    double v1mag = sqrt(x * x + y * y);
    double v2mag = sqrt(v.x * v.x + v.y * v.y);
    double amt = dot / (v1mag * v2mag); //Based of definition of dot product
    //dot product / product of magnitudes gives amt
    if (amt <= -1) {
        return PI;
    } else if (amt >= 1) {
        return 0;
    }
    float tmp = acos(amt);
    return tmp;
}

// normalize divides x and y by magnitude if it has a magnitude.
void ObsVector::normalize()
{
    float m = magnitude();

    if (m > 0) {
        set(x / m, y / m);
    } else {
        set(x, y);
    }
}

// Creates and returns a copy of the ObsVector used as a parameter
ObsVector ObsVector::copy(ObsVector v)
{
    ObsVector copy(v.x, v.y);
    return copy;
}
