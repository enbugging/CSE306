#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "vector.h"

#include <random>
#include <algorithm>
#include <stdio.h>

std::default_random_engine engine;
std::uniform_real_distribution<double> normal(0.0, 1.0);

void sliced_matching(const double* A, const double* B, int W, int H, double* result)
{
	memcpy(result, A, W*H*3 * sizeof(double));
	for (int iter = 0; iter < 100; iter++)
	{
		Vector dir = Vector(normal(engine), normal(engine), normal(engine));
		dir.normalize();
		std::vector<std::pair<double, int> > resultSorted(W * H);
		std::vector<double> BSorted(W * H);
		for (int i = 0; i < W * H; i++)
		{
			double vA = dot(Vector(result[i*3], result[i*3+1], result[i*3+2]), dir);
			double vB = dot(Vector(B[i*3], B[i*3+1], B[i*3+2]), dir);
			resultSorted[i] = std::make_pair(vA, i);
			BSorted[i] = vB;
		}
		std::sort(resultSorted.begin(), resultSorted.end());
		std::sort(BSorted.begin(), BSorted.end());
		for (int i = 0; i < W * H; i++)
		{
			int idx = resultSorted[i].second;
			double delta = BSorted[i] - resultSorted[i].first;
			result[idx*3] += delta * dir[0];
			result[idx*3+1] += delta * dir[1];
			result[idx*3+2] += delta * dir[2];
		}
	}
}

int main() {

	int W, H, C;
	
	//stbi_set_flip_vertically_on_load(true);
	unsigned char *imageTarget = stbi_load(".\\redim.jpg",
                                 &W,
                                 &H,
                                 &C,
                                 STBI_rgb);
	unsigned char *imageSource = stbi_load(".\\imgA.jpg",
                                 &W,
                                 &H,
                                 &C,
                                 STBI_rgb);
	std::vector<double> imageSource_double(W*H*3, 0);
	std::vector<double> imageTarget_double(W*H*3, 0);
	for (int i = 0; i < W * H * 3; i++)
	{
		imageSource_double[i] = imageSource[i];
		imageTarget_double[i] = imageTarget[i];
	}
	std::vector<double> image_result_double(W*H*3);
	sliced_matching(&imageSource_double[0], &imageTarget_double[0], W, H, &image_result_double[0]);
	
	std::vector<unsigned char> image_result(W*H * 3, 0);
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			image_result[(i*W + j) * 3 + 0] = std::min(255., std::max(0., image_result_double[(i*W+j)*3+0]));
			image_result[(i*W + j) * 3 + 1] = std::min(255., std::max(0., image_result_double[(i*W+j)*3+1]));
			image_result[(i*W + j) * 3 + 2] = std::min(255., std::max(0., image_result_double[(i*W+j)*3+2]));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image_result[0], 0);

	return 0;
}