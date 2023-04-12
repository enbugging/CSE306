#ifndef __GEOMETRY_H
#define __GEOMETRY_H
#include "vector.h"
#include "ray.h"

class Object {
public:
    Object(
        const Vector& rho, 
        bool is_mirror = false, 
        bool is_light = false, 
        double refraction_index = 0.0) :
        rho(rho), is_mirror(is_mirror), 
        is_light(is_light), 
        refraction_index(refraction_index) {}
    virtual bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const = 0;
    Vector rho;
    bool is_mirror, is_light;
    double refraction_index;
};
#endif