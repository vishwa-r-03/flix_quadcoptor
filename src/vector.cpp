#include "vector.h"

Vector operator * (const float a, const Vector& b) { return b * a; }
Vector operator + (const float a, const Vector& b) { return b + a; }