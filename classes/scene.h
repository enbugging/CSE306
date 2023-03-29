#ifndef SCENE_H
#define SCENE_H

#include "vector.h"
#include "ray.h"
#include "sphere.h"

#include <random>
#include <cstring>
#include <iostream>
#include <omp.h>

static std::default_random_engine engine[12];
static std::uniform_real_distribution<double> uniform (0, 1);

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

	Vector random_cos(Vector N) 
	{
		int current_thread = omp_get_thread_num();
		double r1 = uniform(engine[current_thread]);
		double r2 = uniform(engine[current_thread]);

		double x = cos(2 * M_PI * r1) * sqrt(1 - r2);
		double y = sin(2 * M_PI * r1) * sqrt(1 - r2);
		double z = sqrt(r2);

		Vector T1, T2;
		if (abs(N[0]) < abs(N[1]) && abs(N[0]) < abs(N[2]))
			T1 = Vector(0, -N[2], N[1]);
		else if (abs(N[1]) < abs(N[0]) && abs(N[1]) < abs(N[2]))
			T1 = Vector(-N[2], 0, N[0]);
		else
			T1 = Vector(-N[1], N[0], 0);
		T2 = cross(N, T1);
		return x*T1 + y*T2 + z*N;
	}

	Vector get_color(const Vector& L, const double r_L, double I, Ray& ray, int bounces = 5)
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
				return get_color(L, r_L, I, reflected_ray, bounces - 1);
			}
			// glass sphere
			if (objects[sphere_id].refraction_index)
			{
				if (this->transparent_mode == "fresnel")
				{
					// Fresnel's law
					Vector M = N;
					double n1 = this->refraction_index; // refractive index of the air
					double n2 = objects[sphere_id].refraction_index; // refractive index of the glass sphere
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
						//std::cerr << "reflection " << bounces << std::endl;
						Vector R = ray.u - 2 * dot(ray.u, M) * M;
						Ray reflected_ray(P + M * 1e-8, R);
						return get_color(L, r_L, I, reflected_ray, bounces - 1);
					}
					else // refraction
					{
						//std::cerr << "refraction " << bounces << std::endl;
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
						return get_color(L, r_L, I, refracted_ray, bounces - 1);
					}
				}
				else 
				{
					// snell's law
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
					return get_color(L, r_L, I, refracted_ray, bounces - 1);
				}
			}

			// direct component
			double d2 = (L - P).norm2();
			Vector lightdir = (L - P);
			lightdir.normalize();

			// Vector D = -lightdir;
			// D.normalize();
			Vector xprime = r_L * random_cos(-lightdir) + L; // this line is supposed to be r_L * random_cos(D) + L
			Vector Nprime = (xprime - L);
			Nprime.normalize();
			Vector omega_i = (xprime - P);
			omega_i.normalize();
			
			Ray shadowRay(P + N * 1e-8, omega_i);
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
				color = I/(4*M_PI*d2) * this->objects[sphere_id].rho / M_PI * std::max(0.0, dot(omega_i, N));
			}

			// indirect component
			Ray r(P+N*1e-8, random_cos(N));
			Vector indirect_color = get_color(L, r_L, I, r, bounces - 1);
			color[0] += indirect_color[0] * this->objects[sphere_id].rho[0];
			color[1] += indirect_color[1] * this->objects[sphere_id].rho[1];
			color[2] += indirect_color[2] * this->objects[sphere_id].rho[2];
		}
		return color;
	}

	Scene(double refraction_index = 1.0, 
		std::string transparent_mode = "snell") : 
		refraction_index(refraction_index), 
		transparent_mode(transparent_mode) {}

	std::vector<Sphere> objects;
	double refraction_index;
	std::string transparent_mode;

};

#endif