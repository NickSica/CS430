#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <string>
#include <argp.h>
#include <fstream>
#include <vector>
#include <cctype>

#include "draw.hpp"

char doc[] = "";
char args_doc[] = "";
struct arguments {
    std::string pscript_file{ "hw1.ps" };
    float scaling_factor{ 1.0f };
    int rot_deg{ 0 };
    int x_translation{ 0 };
    int y_translation{ 0 };
    int x_lower_bound{ 0 };
    int y_lower_bound{ 0 };
    int x_upper_bound{ 499 };
    int y_upper_bound{ 499 };
};

const argp_option options[] {
    {0, 'f', "postscript file", 0, "the input postscript file"},
    {0, 's', "scaling factor", 0, "a float specifying the scaling factor in both dimensions about the world origin"},
    {0, 'r', "rotational degrees", 0, "an integer specifying the number of degrees for couter-clockwise rotation about the world origin"},
    {0, 'm', "x translation", 0, "an integer specifying the translation in the x dimension"},
    {0, 'n', "y translation", 0, "an integer specifying the translation in the y dimension"},
    {0, 'a', "x lower bound", 0, "an integer lower bound in the x dimension of the world window"},
    {0, 'b', "y lower bound", 0, "an integer lower bound in the y dimension of the world window"},
    {0, 'c', "x upper bound", 0, "an integer upper bound in the x dimension of the world window"},
    {0, 'd', "y upper bound", 0, "an integer upper bound in the y dimension of the world window"},
    {0},
};

static error_t parse_opts(int, char *, argp_state *);

struct argp argp { options, parse_opts, args_doc, doc };

void parsePSFile(arguments *, std::vector<std::vector<uint8_t>> *);
void scanConversion();
void clipLines();

#endif
