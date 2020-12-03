#ifndef DRAW_H
#define DRAW_H

#include <iostream>
#include <cmath>
#include <cctype>
#include <vector>
#include <algorithm>
#include <bitset>

struct arguments {
	std::string smf_files[3]{ "bound-sprellpsd.smf", "", "" }; // -f -g -i

	int vw_x_lower_bound{ 0 }; // -j
	int vw_y_lower_bound{ 0 }; // -k
	int vw_x_upper_bound{ 500 }; // -o
	int vw_y_upper_bound{ 500 }; // -p
	int x_res{ 501 };
	int y_res{ 501 };

	float prp_x{ 0.0 }; // -x
	float prp_y{ 0.0 }; // -y
	float prp_z{ 1.0 }; // -z

	float vrp_x{ 0.0 }; // -X
	float vrp_y{ 0.0 }; // -Y
	float vrp_z{ 0.0 }; // -Z

	float vpn_x{ 0.0 }; // -q
	float vpn_y{ 0.0 }; // -r
	float vpn_z{ -1.0 }; // -w

	float vup_x{ 0.0 }; // -Q
	float vup_y{ 1.0 }; // -R
	float vup_z{ 0.0 }; // -W

	float u_min{ -0.7 }; // -u
	float v_min{ -0.7 }; // -v
	float u_max{ 0.7 }; // -U
	float v_max{ 0.7 }; // -V
	bool parallel_proj{ false }; // -P

	float front_plane{ 0.6 }; // -F
	float back_plane{ -0.6 }; // -B

	uint8_t max_val;
};

struct rgb {
	uint8_t red{ 0 };
	uint8_t green{ 0 };
	uint8_t blue{ 0 };
};

struct z_buffer {
	float z;
	rgb color;
};

struct bounds {
	float lower;
	float upper;
};

struct coordinate {
	float x;
	float y;
	float z;
};

struct scan_line {
	std::vector<coordinate> edges;
	std::vector<float> f_intersections;
	std::vector<int> intersections;
};

void applyTransformations(float *, int, arguments *);
void worldToViewport(coordinate *, arguments *, bounds *, bounds *);
void dotProduct(float *, float *, float *, int, int, int);
void normalize(coordinate *, arguments *);
void shearCoord(coordinate *, arguments *, coordinate *, coordinate *, coordinate *);
void scaleCoord(coordinate *, float *);
void rotateViewplane(coordinate *, coordinate *, coordinate *, arguments *);
void translateCoord(coordinate *, coordinate *);
void checkPoint(coordinate *, bounds *, bounds *);
void fillTriangle(std::vector<std::vector<z_buffer>> *, std::vector<coordinate> *,
				 uint8_t [3], float, arguments *args, bounds *, bounds *);

#endif
