#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <string.h>

#include "geometry.h"
#include "vector.h"

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


bool intersect_triangle(
	const Ray& r, 
	const Vector& A, 
	const Vector& B, 
	const Vector& C, 
	const Vector& N_A, 
	const Vector& N_B,
	const Vector& N_C,
	Vector& P, 
	Vector& N, 
	double& t)
{
	Vector e1 = B - A;
	Vector e2 = C - A;
	N = cross(e1, e2);
	double inverse_uN = 1 / (dot(r.u, N));
	t = dot (A - r.O, N) * inverse_uN;
	if (t < 0) return false;
	Vector OAcorssu = cross(A - r.O, r.u);
	double beta = dot(e2, OAcorssu) * inverse_uN;
	double gamma = -dot(e1, OAcorssu) * inverse_uN;
	if (beta < 0 || gamma < 0 || beta + gamma > 1) return false;
	double alpha = 1 - beta - gamma;
	P = t * r.u + r.O;
	N = alpha * N_A + beta * N_B + gamma * N_C; 
	N.normalize();
	return true;
}

class BVH;

class TriangleMesh : public Object {
public:
  ~TriangleMesh() {}
	TriangleMesh(const Vector& rho, bool is_mirror = false, bool is_light = false, double refraction_index = 0.0): bvh(nullptr), Object(rho, is_mirror, is_light, refraction_index) {};
	
	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		if (!f) {
			std::cerr << "Unable to open file " << obj << std::endl;
			return;
		}
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);
	}

	void scale_and_translate(double scaling, const Vector& translate) {
		for (int i = 0; i < indices.size(); i++) {
			vertices[i] = vertices[i] * scaling + translate;
		}
	}
	
	// This is a very slow implementation of ray-triangle intersection, without BVH.
	bool intersect_naive(const Ray& r, Vector& P, Vector& N, double& t, int start_index = 0, int end_index = -1) const {
		if (end_index == -1) end_index = indices.size();
		bool has_inter = false;
		if (end_index - start_index >= 5) printf("Something is wrong! The gap is: %d\n" , end_index - start_index); 
		for (int i = start_index; i < end_index; i++) {
			const Vector& A = vertices[indices[i].vtxi];
			const Vector& B = vertices[indices[i].vtxj];
			const Vector& C = vertices[indices[i].vtxk];
			const Vector& N_A = normals[indices[i].ni];
			const Vector& N_B = normals[indices[i].nj];
			const Vector& N_C = normals[indices[i].nk];

			Vector local_P;
			Vector local_N;
			double local_t;
			bool local_has_inter = intersect_triangle(r, A, B, C, N_A, N_B, N_C, local_P, local_N, local_t);
			if (local_has_inter && local_t < t) {
				has_inter = true;
				t = local_t;
				P = local_P;
				N = local_N;
			}
		}
		//printf("start_index: %d, end_index: %d, res: %d\n", start_index, end_index, has_inter);
		return has_inter;
	}

	void buildBVH();
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const;

	BVH* bvh;
	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
};

class BVH {
public:
    BVH(TriangleMesh* mesh, int start_index = 0, int end_index = -1) {
        if(end_index == -1) end_index = mesh->indices.size();
        this->start_index = start_index;
        this->end_index = end_index;

		// we compute the bounding box
		bx_min = std::numeric_limits<double>::max(), bx_max = -std::numeric_limits<double>::max(),
		by_min = std::numeric_limits<double>::max(), by_max = -std::numeric_limits<double>::max(),
		bz_min = std::numeric_limits<double>::max(), bz_max = -std::numeric_limits<double>::max();
		double
			baryx_min = std::numeric_limits<double>::max(), baryx_max = -std::numeric_limits<double>::max(),
			baryy_min = std::numeric_limits<double>::max(), baryy_max = -std::numeric_limits<double>::max(),
			baryz_min = std::numeric_limits<double>::max(), baryz_max = -std::numeric_limits<double>::max();
		for (int i = start_index; i < end_index; i++)
		{
			bx_min = std::min(bx_min, mesh->vertices[mesh->indices[i].vtxi].data[0]);
			bx_min = std::min(bx_min, mesh->vertices[mesh->indices[i].vtxj].data[0]);
			bx_min = std::min(bx_min, mesh->vertices[mesh->indices[i].vtxk].data[0]);
			bx_max = std::max(bx_max, mesh->vertices[mesh->indices[i].vtxi].data[0]);
			bx_max = std::max(bx_max, mesh->vertices[mesh->indices[i].vtxj].data[0]);
			bx_max = std::max(bx_max, mesh->vertices[mesh->indices[i].vtxk].data[0]);
			
			by_min = std::min(by_min, mesh->vertices[mesh->indices[i].vtxi].data[1]);
			by_min = std::min(by_min, mesh->vertices[mesh->indices[i].vtxj].data[1]);
			by_min = std::min(by_min, mesh->vertices[mesh->indices[i].vtxk].data[1]);
			by_max = std::max(by_max, mesh->vertices[mesh->indices[i].vtxi].data[1]);
			by_max = std::max(by_max, mesh->vertices[mesh->indices[i].vtxj].data[1]);
			by_max = std::max(by_max, mesh->vertices[mesh->indices[i].vtxk].data[1]);

			bz_min = std::min(bz_min, mesh->vertices[mesh->indices[i].vtxi].data[2]);
			bz_min = std::min(bz_min, mesh->vertices[mesh->indices[i].vtxj].data[2]);
			bz_min = std::min(bz_min, mesh->vertices[mesh->indices[i].vtxk].data[2]);
			bz_max = std::max(bz_max, mesh->vertices[mesh->indices[i].vtxi].data[2]);
			bz_max = std::max(bz_max, mesh->vertices[mesh->indices[i].vtxj].data[2]);
			bz_max = std::max(bz_max, mesh->vertices[mesh->indices[i].vtxk].data[2]);

			Vector barycenter = 
				(mesh->vertices[mesh->indices[i].vtxi] + 
				mesh->vertices[mesh->indices[i].vtxj] + 
				mesh->vertices[mesh->indices[i].vtxk]) / 3;
			baryx_min = std::min(baryx_min, barycenter.data[0]);
			baryx_max = std::max(baryx_max, barycenter.data[0]);
			baryy_min = std::min(baryy_min, barycenter.data[1]);
			baryy_max = std::max(baryy_max, barycenter.data[1]);
			baryz_min = std::min(baryz_min, barycenter.data[2]);
			baryz_max = std::max(baryz_max, barycenter.data[2]);
		}
		if (end_index - start_index < 5)
		{
			// the box would be too small, so we don't build a BVH
			child_left = child_right = nullptr;
		}
		else
		{
			// we build the BVH recursively first, then merge the bounding boxes
			int div = 0;
			double diff = baryx_max - baryx_min, mean = (baryx_max + baryx_min) / 2;
			if (baryy_max - baryy_min > diff) div = 1, mean = (baryy_max + baryy_min) / 2, diff = baryy_max - baryy_min;
			if (baryz_max - baryz_min > diff) div = 2, mean = (baryz_max + baryz_min) / 2, diff = baryz_max - baryz_min;
			int pivot_index = start_index;
			for (int i = start_index; i < end_index; i++)
			{
				Vector barycenter = 
					(mesh->vertices[mesh->indices[i].vtxi] + 
					mesh->vertices[mesh->indices[i].vtxj] + 
					mesh->vertices[mesh->indices[i].vtxk]) / 3;
				if (barycenter.data[div] < mean)
				{
					std::swap(mesh->indices[i], mesh->indices[pivot_index]);
					pivot_index++;
				}
			}
			if (pivot_index <= start_index || pivot_index >= end_index)
			{
				child_left = child_right = nullptr;
				return;
			}
			child_left = new BVH(mesh, start_index, pivot_index);
			child_right = new BVH(mesh, pivot_index, end_index);
		}
    }

    bool intersect(const TriangleMesh* mesh, const Ray& r, Vector& P, Vector& N, double& t) const {
		// if there is no bounding box, recursively call intersect() on the children
        if (child_left == nullptr || child_right == nullptr) return mesh->intersect_naive(r, P, N, t, this->start_index, this->end_index);
        
		// else, calculate intersection of the ray with the bounding box
        double
            t1_x = (bx_min - r.O.data[0]) / r.u.data[0], 
            t2_x = (bx_max - r.O.data[0]) / r.u.data[0], 
            t1_y = (by_min - r.O.data[1]) / r.u.data[1],
            t2_y = (by_max - r.O.data[1]) / r.u.data[1],
            t1_z = (bz_min - r.O.data[2]) / r.u.data[2],
            t2_z = (bz_max - r.O.data[2]) / r.u.data[2];
		if (t1_x > t2_x) std::swap(t1_x, t2_x);
		if (t1_y > t2_y) std::swap(t1_y, t2_y);
		if (t1_z > t2_z) std::swap(t1_z, t2_z);
		double
            min_t2 = std::min(t2_x, std::min(t2_y, t2_z)),
            max_t1 = std::max(t1_x, std::max(t1_y, t1_z));
        // if there is no intersection, return false
        if (max_t1 >= min_t2 || max_t1 >= t) return false;

        bool left = child_left->intersect(mesh, r, P, N, t);
        bool right = child_right->intersect(mesh, r, P, N, t);
		//if (left && right) printf("%d %d %d %d %f \n", left, right, start_index, end_index, t);
        return left || right;
    }

    int start_index, end_index;
    double bx_min, bx_max, by_min, by_max, bz_min, bz_max;
    BVH* child_left;
    BVH* child_right;
};

void TriangleMesh::buildBVH() {
	this->bvh = new BVH(this);
}

bool TriangleMesh::intersect(const Ray& r, Vector& P, Vector& N, double& t) const {
	t = std::numeric_limits<double>::max();
	if (this->bvh == nullptr) {
		std::cerr<<"Warning: BVH is not built. Using naive intersection. Performance will be bad."<<std::endl;
		return this->intersect_naive(r, P, N, t);
	}
	return this->bvh->intersect(this, r, P, N, t);
}