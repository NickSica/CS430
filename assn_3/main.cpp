#include "main.h"

int main(int argc, char *argv[])
{
	arguments args;
	argp_parse(&argp, argc, argv, 0, 0, &args);

	std::cerr << "Arguments are " << args.pscript_file << " " << args.scaling_factor << " " << args.rot_deg
			  << " " << args.x_translation << " " << args.y_translation << " " << args.vw_x_lower_bound << " " << args.vw_y_lower_bound << "\n";

	size_t x = (size_t)(args.x_res);
	size_t y = (size_t)(args.y_res);

	std::vector<std::vector<uint8_t>> pixels{ y, std::vector<uint8_t>( x ) };
	parsePSFile(&args, &pixels);
	printPBM(&pixels);
	return 0;
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
		args->ww_x_lower_bound = stoi(arg);
		break;

	case 'b':
		args->ww_y_lower_bound = stoi(arg);
		break;

	case 'c':
		args->ww_x_upper_bound = stoi(arg);
		break;

	case 'd':
		args->ww_y_upper_bound = stoi(arg);
		break;

	case 'j':
		args->vw_x_lower_bound = stoi(arg);
		break;

	case 'k':
		args->vw_y_lower_bound = stoi(arg);
		break;

	case 'o':
		args->vw_x_upper_bound = stoi(arg);
		break;

	case 'p':
		args->vw_y_upper_bound = stoi(arg);
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
	bounds ww_x_bounds{ args->ww_x_lower_bound, args->ww_x_upper_bound };
	bounds ww_y_bounds{ args->ww_y_lower_bound, args->ww_y_upper_bound };
	bounds vw_x_bounds{ args->vw_x_lower_bound, args->vw_x_upper_bound };
	bounds vw_y_bounds{ args->vw_y_lower_bound, args->vw_y_upper_bound };

	std::string line;
	bool cmd_block{ false };
	std::ifstream ps_file{ args->pscript_file };
	std::vector<coordinate> vertices;
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
		if(cmd_block && line.find_first_not_of(WHITESPACE) != std::string::npos)
		{
			// Trim right whitespace
			while(isspace(line[line.length() - 1]))
				line = line.substr(0, line.length() - 1);

			std::string cmd;
			if(line != "stroke")
				cmd = line.substr(line.rfind(' ') + 1);
			else
				cmd = "stroke";

			if(cmd == "Line")
			{
				int length = 4;
				float cmd_parts[length];
				std::string line_parts[length];
				int split_success = splitLines(line, ' ', &line_parts[0], length);
				for(int i = 0; i < length; i++)
					cmd_parts[i] = std::stoi(line_parts[i]);

				applyTransformations(&cmd_parts[0], length, args);
				int draw_line{ clipLine(&cmd_parts[0], &ww_x_bounds, &ww_y_bounds) };
				if(draw_line)
				{
					for(int i = 0; i < length - 1; i += 2)
					{
						coordinate coord{ cmd_parts[i], cmd_parts[i + 1] };
						worldToViewport(&coord, length / 2, args);
						cmd_parts[i] = coord.x;
						cmd_parts[i + 1] = coord.y;
					}

					scanConversion(&cmd_parts[0], pixels, &vw_x_bounds, &vw_y_bounds);
				}
			}
			else if(cmd == "moveto")
			{
				int length = 2;
				vertices.clear();
				float cmd_parts[length];
				std::string line_parts[length];
				int split_success = splitLines(line, ' ', &line_parts[0], length);
				for(int i = 0; i < length; i++)
					cmd_parts[i] = std::stof(line_parts[i]);

				applyTransformations(&cmd_parts[0], length, args);
				coordinate vertex = { cmd_parts[0], cmd_parts[1] };
				vertices.push_back(vertex);
			}
			else if(cmd == "lineto")
			{
				int length = 2;
				float cmd_parts[length];
				std::string line_parts[length];
				int split_success = splitLines(line, ' ', &line_parts[0], length);
				for(int i = 0; i < length; i++)
					cmd_parts[i] = std::stof(line_parts[i]);

				applyTransformations(&cmd_parts[0], length, args);
				coordinate vertex = { cmd_parts[0], cmd_parts[1] };
				vertices.push_back(vertex);
			}
			else if(cmd == "stroke")
			{
				clipPolygon(&vertices, &ww_x_bounds, &ww_y_bounds);
				int length = 2;
				for(int i = 0; i < vertices.size(); ++i)
					worldToViewport(&(vertices[i]), length, args);

				fillPolygon(pixels, &vertices, &vw_x_bounds, &vw_y_bounds);
			}
		}
	}
}

int splitLines(std::string line, char split_char, std::string *line_parts, int num_parts)
{
	int idx{ 0 };
	size_t start{ 0 };
	size_t end{ line.find(split_char) };

	// Splits line using character
	while(end != std::string::npos)
	{
		if(idx < num_parts)
		{
			line_parts[idx] = line.substr(start, end - start);
			start = end + 1;
			end = line.find(split_char, start);
			if(line_parts[idx].find_first_not_of(' ') != std::string::npos)
				idx++;
		}
		else
		{
			std::cerr << "Postscript Error: Too many arguments on line\n";
			return -1;
		}
	}
	return 0;
}

void printPBM(std::vector<std::vector<uint8_t>> *pixels)
{
	int x_size = (*pixels)[0].size();
	int y_size = (*pixels).size();

	std::cout << "P1\n" << x_size << " " << y_size << "\n";
	for(int y = y_size - 1; y >= 0; --y)
	{
	std::string line;
	for(int x = 0; x < x_size; ++x)
		line += std::to_string((*pixels)[y][x]) + " ";
	std::cout << line.substr(0, line.length() - 1) << "\n";
	}
}
