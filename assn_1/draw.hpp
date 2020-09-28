#ifndef DRAW_H
#define DRAW_H

#include <iostream>
#include <cmath>
#include <cctype>
#include <vector>

struct transforms {
    float scaling_factor;
    int rot_degree;
    int x_translation;
    int y_translation;
};

void applyTransformations(int *, transforms *);
void clipLine(int *);
void scanConversion(int *, std::vector<std::vector<uint8_t>> *);

#endif


