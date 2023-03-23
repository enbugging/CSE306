#ifndef SCENE_H
#define SCENE_H

#include "vector.h"
#include "ray.h"
#include "sphere.h"

#include <random>

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

	Vector get_color(const Vector& L, double I, Ray& ray, int bounces = 10)
	{
		if (bounces < 0) return Vector(0, 0, 0);	
		Vector color(0, 0, 0);
		Vector P, N;
		double t;
		int sphere_id;
		
		bool in_shadow = false;

		if (this->intersect(ray, P, N, t, sphere_id)) {
			// if it is a mirror, then we need to reflect the ray
			if (objects[sphere_id].is_mirror)
			{
				Vector R = ray.u - 2 * dot(ray.u, N) * N;
				Ray reflected_ray(P + N * 1e-8, R);
				return get_color(L, I, reflected_ray, bounces - 1);
			}
			/*
			// Snell's law
			if (objects[sphere_id].is_transparent)
			{
				Vector M = N;
				double n1 = 1.0; // refractive index of the air
				double n2 = 1.5; // refractive index of the glass sphere
				double n = n1 / n2;

				double dotprod = dot(ray.u, M);
				if (dotprod > 0)
				{
					// exiting the sphere
					n = 1 / n;
					M = -M;
				}

				Vector tTangent, tNormal;
				double tN;
				tTangent = n * (ray.u - dot(ray.u, M) * M);
				double sintheta = sqr(n) * (1 - sqr(dot(ray.u, M)));
				if (sintheta > 1)
				{
					// total internal reflection
					return color;
				}
				tNormal = -sqrt(1 - sintheta) * M;
				Vector T = tTangent + tNormal;
				Ray refracted_ray(P - M * 1e-8, T);
				return get_color(L, I, refracted_ray, bounces - 1);
			}
			//*/
			if (objects[sphere_id].is_transparent)
			{
				// Fresnel's law
				Vector M = N;
				double n1 = 1.0; // refractive index of the air
				double n2 = 1.5; // refractive index of the glass sphere
				double n = n1 / n2;

				double dotprod = dot(ray.u, M);
				if (dotprod > 0)
				{
					// exiting the sphere
					n = 1 / n;
					M = -M;
				}

				double k = sqr(n1 - n2) / sqr(n1 + n2);
				double R = k + (1 - k) * pow(1 - std::abs(dotprod), 5); // probability of reflection
				std::random_device rd;
    			std::mt19937 gen(rd());
				std::bernoulli_distribution d(R);
				if (d(gen)) // reflection
				{
					Vector R = ray.u - 2 * dot(ray.u, M) * M;
					Ray reflected_ray(P + M * 1e-8, R);
					return get_color(L, I, reflected_ray, bounces - 1);
				}
				else // refraction
				{
					Vector tTangent, tNormal;
					double tN;
					tTangent = n * (ray.u - dot(ray.u, M) * M);
					double sintheta = sqr(n) * (1 - sqr(dot(ray.u, M)));
					if (sintheta > 1)
					{
						// total internal reflection
						return color;
					}
					tNormal = -sqrt(1 - sintheta) * M;
					Vector T = tTangent + tNormal;
					Ray refracted_ray(P - M * 1e-8, T);
					return get_color(L, I, refracted_ray, bounces - 1);
				}
			}

			double d2 = (L - P).norm2();
			Vector lightdir = (L - P);
			lightdir.normalize();
			Ray shadowRay(P + N * 1e-8, lightdir);
			Vector shadow_P, shadow_N;
			double shadow_t;
			int shadow_id;
			if (this->intersect(shadowRay, shadow_P, shadow_N, shadow_t, shadow_id)) {
				if (sqr(shadow_t) < d2) {
					in_shadow = true;
				}
			}
			if (!in_shadow)
			{
				color = I/(4*M_PI*d2) * this->objects[sphere_id].rho / M_PI * std::max(0.0, dot(lightdir, N));
			}
		}
		return color;
	}

	std::vector<Sphere> objects;
};

#endif