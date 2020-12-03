#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <string>
#include <argp.h>
#include <fstream>
#include <array>
#include <vector>
#include <cctype>

#include "draw.h"

char doc[] = "";
char args_doc[] = "";

const argp_option options[] {
	{0, 'f', "smf file", 0, "the input smf file"},
	{0, 'j', "x viewport lower bound", 0, "an integer lower bound in the x dimension of the viewport window"},
	{0, 'k', "y viewport lower bound", 0, "an integer lower bound in the y dimension of the viewport window"},
	{0, 'o', "x viewport upper bound", 0, "an integer upper bound in the x dimension of the viewport window"},
	{0, 'p', "y viewport upper bound", 0, "an integer upper bound in the y dimension of the viewport window"},
	{0, 'x', "x projection reference point", 0, "floating point x of prp in vrc coordinates"},
	{0, 'y', "y projection reference point", 0, "floating point y of prp in vrc coordinates"},
	{0, 'z', "z projection reference point", 0, "floating point z of prp in vrc coordinates"},
	{0, 'X', "x view reference point", 0, "floating point x of vrp in world coordinates"},
	{0, 'Y', "y view reference point", 0, "floating point y of vrp in world coordinates"},
	{0, 'Z', "z view reference point", 0, "floating point z of vrp in world coordinates"},
	{0, 'q', "x view plane normal", 0, "floating point x of vpn in world coordinates"},
	{0, 'r', "y view plane normal", 0, "floating point y of vpn in world coordinates"},
	{0, 'w', "z view plane normal", 0, "floating point z of vpn in world coordinates"},
	{0, 'Q', "x view up vector", 0, "floating point x of vup in world coordinates"},
	{0, 'R', "y view up vector", 0, "floating point y of vup in world coordinates"},
	{0, 'W', "z view up vector", 0, "floating point z of vup in world coordinates"},
	{0, 'u', "u min of vrc window", 0, "floating point u min of the vrc window in vrc coordinates"},
	{0, 'v', "v min of vrc window", 0, "floating point v min of the vrc window in vrc coordinates"},
	{0, 'U', "u max of vrc window", 0, "floating point u max of the vrc window in vrc coordinates"},
	{0, 'V', "v max of vrc window", 0, "floating point v max of the vrc window in vrc coordinates"},
	{0, 'P', 0, 0, "Use parallel projection, otherwise use prespective projection"},
	{0, 'b', 0, 0, "Use backface culling, otherwise don't"},
	{0, 'F', "clip_front", 0, "floating point of front clipping plane"},
	{0, 'B', "clip_back", 0, "floating point of back clipping plane"},
	{0},
};

static error_t parse_opts(int, char *, argp_state *);

struct argp argp { options, parse_opts, args_doc, doc };

void parseSMFFile(arguments *, std::vector<std::vector<uint8_t>> *);
int splitLines(std::string, char, std::string *, int);
void printPBM(std::vector<std::vector<uint8_t>> *);

#endif
