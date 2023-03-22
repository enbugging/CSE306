#ifndef SCENE_H
#define SCENE_H

#include "vector.h"
#include "ray.h"
#include "sphere.h"

class Scene {
public:
	void addSphere(const Sphere& s) {
		objects.push_back(s);
	}

	bool intersect(const Ray& ray, Vector& P, Vector& N, double& t, int& id) {
		bool has_inter = false;
		t = std::numeric_limits<double>::max();
		for (int i = 0; i < objects.size(); i++) {
			Vector localP, localN;
			double localt;
			if (objects[i].intersect(ray, localP, localN, localt)) {
				if (localt < t) {
					t = localt;
					P = localP;
					N = localN;
					id = i;
					has_inter = true;
				}
			}
		}
		return has_inter;
	}

	Vector get_color(Ray& ray, int bounces)
	{
		if (bounces < 0) return Vector(0, 0, 0);	
		Vector color(0, 0, 0);
		Vector P, N;
		Vector L(-10, 20, 40);
		double I = 2E10;
		double t;
		int sphere_id;
		
		bool in_shadow = false;

		double d2 = (L - P).norm2();
		Vector lightdir = (L - P);
		lightdir.normalize();
		if (intersect(ray, P, N, t, sphere_id)) {
			/*
			if (objects[sphere_id].is_mirror) {
				Vector R = ray.u - 2 * dot(ray.u, N) * N;
				Ray mirror_ray(P + 0.0001 * N, R);
				return get_color(mirror_ray, bounces - 1);
			}
			if (objects[sphere_id].is_transparent)
			{
				double n1 = 1;
				double n2 = 1.5;

				Vector nTransp = N;
				if (dot(ray.u, N) < 0) {
					nTransp = -nTransp;
					std::swap(n1, n2);
				}
				Vector tTangent, tNormal;
				tTangent = 
					n1 / n2 * 
					(ray.u - dot(ray.u, nTransp) * nTransp);
				double rad = 
					1 - sqr(n1/n2) * (1 - sqr(dot(ray.u, nTransp)));
				if (rad < 0)
				{
					Vector mirrorDirection = 
						ray.u - 2 * dot(ray.u, nTransp) * nTransp;
					Ray mirrorRay(P + 0.0001 * nTransp, mirrorDirection);
					return get_color(mirrorRay, bounces - 1);
				}
				tNormal = -sqrt(rad) * nTransp;
				Ray refractedRay(P - 0.0001 * nTransp, tTangent + tNormal);
				return get_color(refractedRay, bounces - 1);
			}
			//*/
			color = I/(4*M_PI*d2) * objects[sphere_id].rho / M_PI * dot(N, lightdir);
			return color;
		}
		else
		{
			Ray shadowRay(P + 0.0001 * N, lightdir);

			Vector shadowP, shadowN;
			double shadowt;
			int shadow_id;
			if (intersect(shadowRay, shadowP, shadowN, shadowt, shadow_id))
			{
				if (sqr(shadowt) < d2) {
					in_shadow = true;
				}
				color = I/(4*M_PI*d2) * objects[sphere_id].rho / M_PI * dot(N, lightdir);
			}
			if (not in_shadow) return color;
			else return Vector(0,0,0);
		}
	}
	std::vector<Sphere> objects;
};

#endif