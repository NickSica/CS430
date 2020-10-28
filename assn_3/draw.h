#ifndef DRAW_H
#define DRAW_H

#include <iostream>
#include <cmath>
#include <cctype>
#include <vector>
#include <algorithm>
#include <bitset>

struct arguments {
	std::string pscript_file{ "hw3.ps" };
	float scaling_factor{ 1.0f };
	int rot_deg{ 0 };
	int x_translation{ 0 };
	int y_translation{ 0 };
	int ww_x_lower_bound{ 0 };
	int ww_y_lower_bound{ 0 };
	int ww_x_upper_bound{ 250 };
	int ww_y_upper_bound{ 250 };
	int vw_x_lower_bound{ 0 };
	int vw_y_lower_bound{ 0 };
	int vw_x_upper_bound{ 200 };
	int vw_y_upper_bound{ 200 };
	int x_res { 501 };
	int y_res { 501 };
};

struct bounds {
	int lower;
	int upper;
};

struct coordinate {
	float x;
	float y;
};

struct edge {
	int x1;
	int y1;
	int x2;
	int y2;
};

void applyTransformations(float *, int, arguments *);
void worldToViewport(coordinate *, int, arguments *);
void scaleCoord(coordinate *, float *);
void rotateCoord(coordinate *, int);
void translateCoord(coordinate *, int, int);
int clipLine(float *, bounds *, bounds *);
void clipPolygon(std::vector<coordinate> *, bounds *, bounds *);
void fillPolygon(std::vector<std::vector<uint8_t>> *, std::vector<coordinate> *, bounds *, bounds *);
void scanConversion(float *, std::vector<std::vector<uint8_t>> *, bounds *, bounds *);

#endif
