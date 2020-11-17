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
	{0, 'f', "postscript file", 0, "the input postscript file"},
	{0, 's', "scaling factor", 0, "a float specifying the scaling factor in both dimensions about the world origin"},
	{0, 'r', "rotational degrees", 0, "an integer specifying the number of degrees for couter-clockwise rotation about the world origin"},
	{0, 'm', "x translation", 0, "an integer specifying the translation in the x dimension"},
	{0, 'n', "y translation", 0, "an integer specifying the translation in the y dimension"},
	{0, 'a', "x world lower bound", 0, "an integer lower bound in the x dimension of the world window"},
	{0, 'b', "y world lower bound", 0, "an integer lower bound in the y dimension of the world window"},
	{0, 'c', "x world upper bound", 0, "an integer upper bound in the x dimension of the world window"},
	{0, 'd', "y world upper bound", 0, "an integer upper bound in the y dimension of the world window"},
	{0, 'j', "x viewport lower bound", 0, "an integer lower bound in the x dimension of the viewport window"},
	{0, 'k', "y viewport lower bound", 0, "an integer lower bound in the y dimension of the viewport window"},
	{0, 'o', "x viewport upper bound", 0, "an integer upper bound in the x dimension of the viewport window"},
	{0, 'p', "y viewport upper bound", 0, "an integer upper bound in the y dimension of the viewport window"},
	{0, 'N', "line_segments", 0, "an integer specifying the number of line segments used to represent the curve"},
	{0},
};

enum Command { none, line, curve, polygon };

static error_t parse_opts(int, char *, argp_state *);

struct argp argp { options, parse_opts, args_doc, doc };

void parsePSFile(arguments *, std::vector<std::vector<uint8_t>> *);
int splitLines(std::string, char, std::string *, int);
void printPBM(std::vector<std::vector<uint8_t>> *);

#endif
