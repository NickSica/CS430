#include "main.h"

int main(int argc, char *argv[])
{
	arguments args;
	argp_parse(&argp, argc, argv, 0, 0, &args);

	size_t x = (size_t)(args.x_res);
	size_t y = (size_t)(args.y_res);
	args.max_val = 20;//255;

	std::vector<std::vector<z_buffer>> z_buff( y, std::vector<z_buffer>( x ) );
	for(int i = 0; i < y; ++i)
		for(int j = 0; j < x; ++j)
			z_buff[i][j].z = -1;

	for(int i = 0; i < 3; ++i)
	{
		uint8_t color[3]{ 0, 0, 0 };
		color[i] = args.max_val;

		if(args.smf_files[i] == "")
			continue;

		parseSMFFile(&args, &z_buff, args.smf_files[i], color);
	}

	printPPM(&z_buff, args.max_val);
	return 0;
}

static error_t parse_opts(int key, char *arg_char, argp_state *state)
{
	if(arg_char == NULL && key != 'P')
		return 0;

	arguments *args = (arguments *)state->input;
	std::string arg;
	if(key != 'P')
		arg = arg_char;

	switch(key)
	{
	case 'f':
		args->smf_files[0] = arg;
		break;

	case 'g':
		args->smf_files[1] = arg;
		break;

	case 'i':
		args->smf_files[2] = arg;
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

	case 'x':
		args->prp_x = stof(arg);
		break;

	case 'y':
		args->prp_y = stof(arg);
		break;

	case 'z':
		args->prp_z = stof(arg);
		break;

	case 'X':
		args->vrp_x = stof(arg);
		break;

	case 'Y':
		args->vrp_y = stof(arg);
		break;

	case 'Z':
		args->vrp_z = stof(arg);
		break;

	case 'q':
		args->vpn_x = stof(arg);
		break;

	case 'r':
		args->vpn_y = stof(arg);
		break;

	case 'w':
		args->vpn_z = stof(arg);
		break;

	case 'Q':
		args->vup_x = stof(arg);
		break;

	case 'R':
		args->vup_y = stof(arg);
		break;

	case 'W':
		args->vup_z = stof(arg);
		break;

	case 'u':
		args->u_min = stof(arg);
		break;

	case 'v':
		args->v_min = stof(arg);
		break;

	case 'U':
		args->u_max = stof(arg);
		break;

	case 'V':
		args->v_max = stof(arg);
		break;

	case 'P':
		args->parallel_proj = true;
		break;

	case 'F':
		args->front_plane = stof(arg);
		break;

	case 'B':
		args->back_plane = stof(arg);
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

void parseSMFFile(arguments *args, std::vector<std::vector<z_buffer>> *z_buff, std::string file_name, uint8_t color[3])
{
	std::cerr << "Parsing file " << file_name << "\n";
	const std::string WHITESPACE = " \t\r\n\f\v";
	bounds vw_x_bounds{ (float)args->vw_x_lower_bound, (float)args->vw_x_upper_bound };
	bounds vw_y_bounds{ (float)args->vw_y_lower_bound, (float)args->vw_y_upper_bound };

	std::ifstream smf_file{ file_name };
	std::string line;
	bool cmd_block{ false };
	std::vector<coordinate> vertices;
	while(std::getline(smf_file, line))
	{
		std::cerr << line << "\n";
		// Trim right whitespace
		while(isspace(line[line.length() - 1]))
			line = line.substr(0, line.length() - 1);

		if(line[0] == '#')
		{
			if(line[1] == '$')
			{
				std::string config_cmd = line.substr(2, line.find(' '));
				if (config_cmd == "vertices")
				{
					int num_vertices = stoi(line.substr(line.rfind(' ')));
					vertices.reserve(num_vertices);
				}
			}
			continue;
		}

		// In the command block and it has more than just whitespace
		if(line.find_first_not_of(WHITESPACE) != std::string::npos)
		{
			std::string cmd;
			size_t cmd_idx = line.find(' ');
			cmd = line.substr(0, line.find(' '));
			std::cerr << "Command: " << cmd << "\n";

			int length = 3;
			std::string line_parts[length];
			line = line.substr(cmd_idx + 1) + ' ';
			int split_success = splitLines(line, ' ', &line_parts[0], length);

			if(cmd == "v")
			{
				coordinate vertex{};
				vertex.x = stof(line_parts[0]);
				vertex.y = stof(line_parts[1]);
				vertex.z = stof(line_parts[2]);

				normalize(&vertex, args);
				vertices.push_back(vertex);
			}
			else if(cmd == "f")
			{
				length++;
				std::vector<coordinate> face;
				face.resize(length);
				for (int i = 0; i < length - 1; ++i)
				{
					int vert_idx = stoi(line_parts[i]) - 1;
					face[i].x = vertices[vert_idx].x;
					face[i].y = vertices[vert_idx].y;
					face[i].z = vertices[vert_idx].z;
				}
				face[length - 1].x = face[0].x;
				face[length - 1].y = face[0].y;
				face[length - 1].z = face[0].z;

				float z_min{ (args->prp_z - args->front_plane) / (args->back_plane - args->prp_z) };
				if(args->parallel_proj)
					z_min = 0;

				float z_proj{ (args->prp_z) / (args->back_plane - args->prp_z) };
				for(int i = 0; i < length; ++i)
				{
					if(!args->parallel_proj)
					{
						float z_d = face[i].z / z_proj;
						face[i].x /= z_d;
						face[i].y /= z_d;
					}
				}

				bounds window_x_bounds;
				bounds window_y_bounds;
				if(args->parallel_proj)
				{
					window_x_bounds = { -1, 1 };
					window_y_bounds = { -1, 1 };
				}
				else
				{
					float abs_z_proj = std::abs(z_proj);
					window_x_bounds = { -abs_z_proj, abs_z_proj };
					window_y_bounds = { -abs_z_proj, abs_z_proj };
				}

				for(int i = 0; i < face.size() - 1; ++i)
					worldToViewport(&(face[i]), args, &window_x_bounds, &window_y_bounds);
				face[face.size() - 1] = face[0];

				//for(int i = 0; i < face.size(); ++i)
				//	std::cerr << "(" << face[i].x << ", " << face[i].y << ", " << face[i].z << ")\n";
				fillTriangle(z_buff, &face, color, z_min, args, &vw_x_bounds, &vw_y_bounds);
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
			std::cerr << "SMF Error: Too many arguments on line\n";
			return -1;
		}
	}
	return 0;
}

void printPPM(std::vector<std::vector<z_buffer>> *z_buff, int max_val)
{
	std::cerr << "Printing PBM start" << '\n';
	int x_size = (*z_buff)[0].size();
	int y_size = (*z_buff).size();

	std::cout << "P3\n" << x_size << " " << y_size << "\n" << max_val << "\n";
	int pad_len = std::to_string(max_val).length();
	for(int y = y_size - 1; y >= 0; --y)
	{
		std::string line;
		for(int x = 0; x < x_size; ++x)
		{
			rgb color{ (*z_buff)[y][x].color };
			std::string col_string{ std::to_string(color.red) };
			col_string.insert(col_string.begin(), pad_len - col_string.length(), ' ');
			line += col_string;

			pad_len++;
			col_string = std::to_string(color.green);
			col_string.insert(col_string.begin(), pad_len - col_string.length(), ' ');
			line += col_string;

			col_string = std::to_string(color.blue);
			col_string.insert(col_string.begin(), pad_len - col_string.length(), ' ');
			line += col_string + "   ";
			pad_len--;
		}
		std::cout << line.substr(0, line.length() - 3) << "\n";
	}
}
