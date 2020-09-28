#include "main.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    arguments args;
    argp_parse(&argp, argc, argv, 0, 0, &args);

    size_t cols = (size_t)(args.x_upper_bound - args.x_lower_bound);
    size_t rows = (size_t)(args.y_upper_bound - args.y_lower_bound);

    std::vector<std::vector<uint8_t>> pixels{ rows, std::vector<uint8_t>( cols ) };
    parsePSFile(&args, &pixels);
}

static error_t parse_opts(int key, char *arg_char, argp_state *state)
{
    if(arg_char == NULL)
	return 0;

    arguments *args = (arguments *)state->input;
    std::string arg = arg_char;

    switch(key)
    {
    case 'f':
	args->pscript_file = arg_char;
	break;

    case 's':
	args->scaling_factor = stof(arg);
	break;

    case 'r':
	args->rot_deg = stoi(arg);
	break;

    case 'm':
	args->x_translation = stoi(arg);
	break;

    case 'n':
	args->y_translation = stoi(arg);
	break;

    case 'a':
	args->x_lower_bound = stoi(arg);
	break;

    case 'b':
	args->y_lower_bound = stoi(arg);
	break;

    case 'c':
	args->x_upper_bound = stoi(arg);
	break;

    case 'd':
	args->y_upper_bound = stoi(arg);
	break;
	
    case ARGP_KEY_ARG:
	if(state->arg_num > 9)
	    argp_usage(state);
	break;
	
    default:
	return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

void parsePSFile(arguments *args, std::vector<std::vector<uint8_t>> *pixels)
{
    std::cerr << "Parsing file " << args->pscript_file << "\n";
    const std::string WHITESPACE = " \t\r\n\f\v";

    transforms transforms{ args->scaling_factor, args->rot_deg, args->x_translation, args->y_translation };
    std::string line;
    bool cmd_block{ false };
    std::ifstream ps_file{ args->pscript_file };
    if(ps_file.is_open())
    {
	while(std::getline(ps_file, line))
	{
	    std::cerr << line << "\n";
	    if(line.substr(0, 8) == "%%%BEGIN")
	    {
		cmd_block = true;
		continue;
	    }

	    if(line.substr(0, 6) == "%%%END")
	    {
		cmd_block = false;
		continue;
	    }

	    // In the command block and it has more than just whitespace
	    if(cmd_block && !isspace(line[0]))
	    {
		// Trim right whitespace
		while(isspace(line[line.length() - 1]))
		    line = line.substr(0, line.length() - 2);

		std::string cmd{ line.substr(line.rfind(' ') + 1) };
		if(cmd == "Line")
		{
		    int cmd_parts[4];
		    int idx{ 0 };
		    size_t start{ 0 };
		    size_t end{ line.find(' ') };

		    // Splits line using whitespace
		    while(end != std::string::npos)
		    {
			cmd_parts[idx] = stoi(line.substr(start, end - start));
			start = end + 1;
			end = line.find(' ', start);
			idx++;
		    }

		    applyTransformations(&cmd_parts[0], &transforms);
		    clipLines();
                    scanConversion(&cmd_parts[0], &pixels);
		}
	    }
	}
	ps_file.close();
    }
}


