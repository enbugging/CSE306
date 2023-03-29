#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "classes/vector.h"
#include "classes/sphere.h"
#include "classes/ray.h"
#include "classes/scene.h"

#include <cmath>
#include <ctime>
#include <iostream>


int main() {
	int W = 512;
	int H = 512;

	Scene scene(1.0, "snell");

	Sphere S(Vector(0, 0, 0), 10.0, Vector(1, 1, 1));
	scene.addSphere(S);
	Sphere S_interior(Vector(0, 0, 0), 9.0, Vector(0, 0.5, 1), false, 1);
	scene.addSphere(S_interior);
	
	Sphere left_wall(Vector(-1000, 0, 0), 940.0, Vector(0.5, 0.8, 0.1));
	scene.addSphere(left_wall);
	Sphere right_wall(Vector(1000, 0, 0), 940.0, Vector(0.9, 0.2, 0.3));
	scene.addSphere(right_wall);
	Sphere ceilling(Vector(0, 1000, 0), 940.0, Vector(0.3, 0.5, 0.3));
	scene.addSphere(ceilling);
	Sphere floor(Vector(0, -1000, 0), 990.0, Vector(0.6, 0.5, 0.7));
	scene.addSphere(floor);
	Sphere front_wall(Vector(0, 0, -1000), 940.0, Vector(0.1, 0.6, 0.7));
	scene.addSphere(front_wall);
	Sphere behind_wall(Vector(0, 0, 1000), 940.0, Vector(0.8, 0.2, 0.9));
	scene.addSphere(behind_wall);

	Vector camera_center(0, 0, 35);
	double alpha = 60. / 180. * M_PI;
	Vector L(-10, 20, 40);
	double I = 2E10;

	int number_of_samples = 20;

	std::clock_t t = std::clock();
	std::vector<unsigned char> image(W * H * 3, 0);
	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			
			Vector ray_dir;
			ray_dir[0] =  j - W / 2. + .5;
			ray_dir[1] = -i + H / 2. - .5;
			ray_dir[2] = -W / (2. * tan(alpha / 2.));
			ray_dir.normalize();
			Ray r(camera_center, ray_dir);
			Vector color(0, 0, 0);
			for (int sample = 0; sample < number_of_samples; sample++)
			{
				color = color + scene.get_color(L, I, r);
			}
			color = color / number_of_samples;
			image[3 * (i * W + j) + 0] = std::min(255., std::pow(color[0], 1./2.2));
			image[3 * (i * W + j) + 1] = std::min(255., std::pow(color[1], 1./2.2));
			image[3 * (i * W + j) + 2] = std::min(255., std::pow(color[2], 1./2.2));
		}
	}
	t = std::clock() - t;
	std::cerr << "Time elapsed: " << ((float)t)/CLOCKS_PER_SEC << std::endl;
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}