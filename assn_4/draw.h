#ifndef DRAW_H
#define DRAW_H

#include <iostream>
#include <cmath>
#include <cctype>
#include <vector>
#include <algorithm>
#include <bitset>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct arguments {
	std::string smf_file{ "bound-lo-sphere.smf" }; // -f
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

	float clip_front{ 0.6 }; // -F
	float clip_back{ -0.6 }; // -B
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

struct edge {
	float x1;
	float y1;
	float z1;
	float x2;
	float y2;
	float z2;
};

void applyTransformations(float *, int, arguments *);
void worldToViewport(coordinate *, arguments *, bounds *, bounds *);
void normalize(coordinate *, arguments *);
void shearCoord(coordinate *, arguments *, coordinate *, coordinate *, coordinate *);
void scaleCoord(coordinate *, float *);
void rotateViewplane(Eigen::Matrix<float, 3, 1> *, Eigen::Matrix<float, 3, 1> *, Eigen::Matrix<float, 3, 1> *, arguments *);
void translateCoord(coordinate *, coordinate *);
int clipLine(float *, bounds *, bounds *);
void clipPolygon(std::vector<coordinate> *, bounds *, bounds *);
void fillPolygon(std::vector<std::vector<uint8_t>> *, std::vector<coordinate> *, bounds *, bounds *);
bool trivialReject(std::vector<coordinate> *, int, bool);
void checkPoint(coordinate *, bounds *, bounds *);
void scanConversion(float *, std::vector<std::vector<uint8_t>> *, bounds *, bounds *);

#endif
