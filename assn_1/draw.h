#ifndef DRAW_H
#define DRAW_H

#include <iostream>
#include <cmath>
#include <cctype>
#include <vector>
#include <bitset>

struct transforms {
    float scaling_factor;
    int rot_degree;
    int x_translation;
    int y_translation;
};

struct bounds {
    int lower;
    int upper;
};

void applyTransformations(float *, transforms *);
int clipLine(float *, bounds *, bounds *);
void scanConversion(float *, std::vector<std::vector<uint8_t>> *, bounds *, bounds *);

#endif
