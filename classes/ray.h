#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray {
public:
	Ray(const Vector& O, const Vector& u) : O(O), u(u) {}
	Vector O;
	Vector u;
};

#endif