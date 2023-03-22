#ifndef SPHERE_H
#define SPHERE_H

#include "vector.h"
#include "ray.h"

class Sphere {
public:
	Sphere(
        const Vector& C, 
        double R, 
        const Vector& rho, 
        bool is_mirror = false, 
        bool is_transparent = false) : 
        C(C), 
        R(R), 
        rho(rho), 
        is_mirror(is_mirror), 
        is_transparent(is_transparent) {}

	bool intersect(const Ray& r, Vector& P, Vector& N, double& t) {
		double delta = sqr(dot(r.u, r.O - C)) - (r.O - C).norm2() + sqr(R);

		if (delta >= 0) {
			double t1 = dot(r.u, C - r.O) - sqrt(delta);
			double t2 = dot(r.u, C - r.O) + sqrt(delta);
			if (t2 < 0) return false;
			if (t1 >= 0) {
				t = t1;
			}
			else {
				t = t2;
			}
			P = r.O + t * r.u;
			N = P - C;
			N.normalize();
			return true;
		}
		else {
			return false;
		}
	}

	Vector C;
	double R;
	Vector rho;
	bool is_mirror, is_transparent;
};

#endif