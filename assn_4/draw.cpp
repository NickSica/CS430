#include "draw.h"

void worldToViewport(coordinate *coord, arguments *args)
{
	float x_scale = (float)(args->vw_x_upper_bound - args->vw_x_lower_bound) / (float)(args->u_max - args->u_min);
	float y_scale = (float)(args->vw_y_upper_bound - args->vw_y_lower_bound) / (float)(args->v_max - args->v_min);
	float scaling_factors[2] { x_scale, y_scale };
	// First translate to origin
	coordinate tran_coord{ -args->u_min, -args->v_min, 0 };
	translateCoord(coord, &tran_coord);

	// Second scale to viewport size
	scaleCoord(coord, scaling_factors);

	// Third translate to final position
	tran_coord = {
		.x = (float)args->vw_x_lower_bound,
		.y = (float)args->vw_y_lower_bound,
		.z = 0 };
	translateCoord(coord, &tran_coord);
}

void perspectiveNorm(coordinate *coord, arguments *args)
{
	coordinate vrp_neg{ -args->vrp_x, -args->vrp_y, -args->vrp_z };
	translateCoord(coord, &vrp_neg);

	rotateViewplane(coord, args);

	coordinate prp_neg{ -args->prp_x, -args->prp_y, -args->prp_z };
	translateCoord(coord, &prp_neg);

	float u_shear = (0.5 * (args->u_max + args->u_min) - args->prp_x) / args->prp_z;
	float v_shear = (0.5 * (args->v_max + args->v_min) - args->prp_y) / args->prp_z;
	coordinate x_shear{ 1, 0, u_shear };
	coordinate y_shear{ 0, 1, v_shear };
	coordinate z_shear{ 0, 0, 1 };
	shearCoord(coord, args, &x_shear, &y_shear, &z_shear);

	float u_diff = 1 / (args->u_max - args->u_min);
	float v_diff = 1 / (args->v_max - args->v_min);
	float prp_diff = 1 / (args->prp_z - args->clip_back);
	float scaling_factors[3]{ 2 * args->prp_z * u_diff * prp_diff,
		2 * args->prp_z * v_diff * prp_diff,
		prp_diff };
	scaleCoord(coord, scaling_factors);
}

void parallelNorm(coordinate *coord, arguments *args)
{
	coordinate vrp_neg{ -args->vrp_x, -args->vrp_y, -args->vrp_z };
	translateCoord(coord, &vrp_neg);

	rotateViewplane(coord, args);

	float u_shear = (0.5 * (args->u_max + args->u_min) - args->prp_x) / args->prp_z;
	float v_shear = (0.5 * (args->v_max + args->v_min) - args->prp_y) / args->prp_z;
	coordinate x_shear{ 1, 0, u_shear };
	coordinate y_shear{ 0, 1, v_shear };
	coordinate z_shear{ 0, 0, 1 };
	shearCoord(coord, args, &x_shear, &y_shear, &z_shear);

	float x_tran{ -(args->u_max + args->u_min) / 2 };
	float y_tran{ -(args->v_max + args->v_min) / 2 };
	float z_tran{ -args->clip_front };

	float x_scale{ 2 / (args->u_max - args->u_min) };
	float y_scale{ 2 / (args->v_max - args->v_min) };
	float z_scale{ 1 / (args->clip_front - args->clip_back) };
	float scaling_factors[3] { x_scale, y_scale, z_scale };
	scaleCoord(coord, scaling_factors);
}

void shearCoord(coordinate *coord, arguments *args, coordinate *x_shear, coordinate *y_shear, coordinate *z_shear)
{
	coord->x = coord->x + x_shear->y * coord->y + x_shear->z * coord->z;
	coord->x = y_shear->x * coord->x + coord->y + y_shear->z * coord->z;
	coord->x = z_shear->x * coord->x + z_shear->y * coord->y + coord->z;
}

void scaleCoord(coordinate *coord, float *scaling_factors)
{
	if(scaling_factors[0] != 1.0)
		coord->x = coord->x * scaling_factors[0];

	if(scaling_factors[1] != 1.0)
		coord->y = coord->y * scaling_factors[1];

	if(scaling_factors[0] != 1.0)
		coord->z = coord->z * scaling_factors[2];
}

void rotateViewplane(coordinate *coord, arguments *args)
{
	coordinate rot_x;
	coordinate rot_y;
	coordinate rot_z;
	if(args->vpn_x != 0)
		rot_z.x = args->vpn_x / std::abs(args->vpn_x);
	else
		rot_z.x = 0;

	if(args->vpn_y != 0)
		rot_z.y = args->vpn_y / std::abs(args->vpn_y);
	else
		rot_z.y = 0;

	if(args->vpn_z != 0)
		rot_z.z = args->vpn_z / std::abs(args->vpn_z);
	else
		rot_z.z = 0;

	float x_cross = args->vup_y * rot_z.z - args->vup_z * rot_z.y;
	float y_cross = args->vup_z * rot_z.x - args->vup_x * rot_z.z;
	float z_cross = args->vup_x * rot_z.y - args->vup_y * rot_z.x;

	if(x_cross != 0)
		rot_x.x = x_cross / std::abs(x_cross);
	else
		rot_x.x = 0;

	if(y_cross != 0)
		rot_x.y = y_cross / std::abs(y_cross);
	else
		rot_x.y = 0;

	if(z_cross != 0)
		rot_x.z = z_cross / std::abs(z_cross);
	else
		rot_x.z = 0;

	rot_y.x = rot_z.y * rot_x.z - rot_z.z * rot_x.y;
	rot_y.y = rot_z.z * rot_x.x - rot_z.x * rot_x.z;
	rot_y.z = rot_z.x * rot_x.y - rot_z.y * rot_x.x;

	float x = coord->x;
	float y = coord->y;
	float z = coord->z;

	coord->x = rot_x.x * x + rot_x.y * y + rot_x.z * z;
	coord->y = rot_y.x * x + rot_y.y * y + rot_y.z * z;
	coord->z = rot_z.x * x + rot_z.y * y + rot_z.z * z;
}

void translateCoord(coordinate *coord, coordinate *tran_coord)
{
	coord->x = coord->x + tran_coord->x;
	coord->y = coord->y + tran_coord->y;
	coord->z = coord->z + tran_coord->z;
}

// Uses Cohen-Sutherland algorithm to clip lines to world view, returns 1 if it should draw the line
int clipLine(float *cmd_parts, bounds *x_bounds, bounds *y_bounds)
{
	std::cerr << "Line Clipping Algorithm start\n";
	uint8_t codes[2] = { 0, 0 };

	for(int i = 0; i < 4; i += 2)
	{
		if(cmd_parts[i]		< x_bounds->lower) codes[i >> 1] |= 0b0001;
		if(cmd_parts[i]		> x_bounds->upper) codes[i >> 1] |= 0b0010;
		if(cmd_parts[i + 1] < y_bounds->lower) codes[i >> 1] |= 0b0100;
		if(cmd_parts[i + 1] > y_bounds->upper) codes[i >> 1] |= 0b1000;
	}

	// Clip until points can be trivially accepted or rejected
	int point_idx = 0;
	while((codes[0] | codes[1]) != 0b0000 && (codes[0] & codes[1]) == 0b0000)
	{
		float x1 = cmd_parts[0];
		float y1 = cmd_parts[1];
		float x2 = cmd_parts[2];
		float y2 = cmd_parts[3];
		int idx{ point_idx >> 1 };
		if((codes[idx] & 0b0001) == 0b0001)
		{
			float xc = x_bounds->lower;
			cmd_parts[point_idx] = xc;
			cmd_parts[point_idx + 1] = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
		}
		else if((codes[idx] & 0b0010) == 0b0010)
		{
			float xc = x_bounds->upper;
			cmd_parts[point_idx] = xc;
			cmd_parts[point_idx + 1] = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
		}
		else if((codes[idx] & 0b0100) == 0b0100)
		{
			float yc = y_bounds->lower;
			cmd_parts[point_idx] = (x2 - x1) * (yc - y1) / (y2 - y1)  + x1;
			cmd_parts[point_idx + 1] = yc;
		}
		else if((codes[idx] & 0b1000) == 0b1000)
		{
			float yc = y_bounds->upper;
			cmd_parts[point_idx] = (x2 - x1) * (yc - y1) / (y2 - y1) + x1;
			cmd_parts[point_idx + 1] = yc;
		}

		codes[idx] = 0;
		if(cmd_parts[point_idx]		< x_bounds->lower) codes[idx] |= 0b0001;
		if(cmd_parts[point_idx]		> x_bounds->upper) codes[idx] |= 0b0010;
		if(cmd_parts[point_idx + 1] < y_bounds->lower) codes[idx] |= 0b0100;
		if(cmd_parts[point_idx + 1] > y_bounds->upper) codes[idx] |= 0b1000;

		if(point_idx == 0) point_idx = 2;
		else point_idx = 0;
	}

	// Lie completely outside view window
	if((codes[0] & codes[1]) != 0b0000)
		return 0;

	// Lie completely inside view window
	return 1;
}

// Clip the polygon with Sutherland-Hodgman
void clipPolygon(std::vector<coordinate> *vertices, bounds *x_bounds, bounds *y_bounds)
{
	std::cerr << "Polygon Clipping start" << '\n';
	std::vector<coordinate> new_vertices(vertices->size() * 2);
	bounds *bounds = x_bounds;

	// Each loop iteration is another bound check- 0 checks against x lower, 1: x upper, 2: y lower, 3: x lower
	for(int i = 0; i < 4; i++)
	{
		int idx{ 0 };
		float x1{ (*vertices)[0].x };
		float y1{ (*vertices)[0].y };
		switch(i)
		{
		case 0:
			if(x1 >= bounds->lower)
			{
				new_vertices[idx] = (*vertices)[0];
				idx++;
			}
			break;
		case 1:
			if(x1 <= bounds->upper)
			{
				new_vertices[idx] = (*vertices)[0];
				idx++;
			}
			break;
		case 2:
			if(y1 >= bounds->lower)
			{
				new_vertices[idx] = (*vertices)[0];
				idx++;
			}
			break;
		case 3:
			if(y1 <= bounds->upper)
			{
				new_vertices[idx] = (*vertices)[0];
				idx++;
			}
			break;
		}

		// Clip against x lower bound
		for(int j = 0; j < vertices->size() - 1; ++j)
		{
			float x1{ (*vertices)[j].x };
			float y1{ (*vertices)[j].y };
			float x2{ (*vertices)[j + 1].x };
			float y2{ (*vertices)[j + 1].y };

			uint8_t v1_in;
			uint8_t v2_in;
			switch(i)
			{
			case 0:
				v1_in = x1 >= bounds->lower;
				v2_in = x2 >= bounds->lower;
				break;
			case 1:
				v1_in = x1 <= bounds->upper;
				v2_in = x2 <= bounds->upper;
				break;
			case 2:
				v1_in = y1 >= bounds->lower;
				v2_in = y2 >= bounds->lower;
				break;
			case 3:
				v1_in = y1 <= bounds->upper;
				v2_in = y2 <= bounds->upper;
				break;
			}

			float xc{ i == 0 ? (float)bounds->lower : (float)bounds->upper };
			float yc{ i == 2 ? (float)bounds->lower : (float)bounds->upper };
			if(v1_in & v2_in)
			{
				new_vertices[idx] = { x2, y2 };
				idx++;
			}
			else if(v1_in && !v2_in)
			{
				std::cerr << "Clipping (" << x2 << ", " << y2 << ")";
				if(i < 2)
				{
					y2 = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
					x2 = xc;
				}
				else
				{
					x2 = (x2 - x1) * (yc - y1) / (y2 - y1)	+ x1;
					y2 = yc;
				}
				new_vertices[idx] = { x2, y2 };
				idx++;
			}
			else if(!v1_in && v2_in)
			{
				std::cerr << "Clipping (" << x1 << ", " << y1 << ")";
				if(i < 2)
				{
					y1 = (y1 - y2) * (xc - x2) / (x1 - x2) + y2;
					x1 = xc;
				}
				else
				{
					x1 = (x1 - x2) * (yc - y2) / (y1 - y2)	+ x2;
					y1 = yc;
				}
				new_vertices[idx] = { x1, y1 };
				idx++;

				new_vertices[idx] = { x2, y2 };
				idx++;
			}
		}

		// Copy changes over and reset everything
		*vertices = new_vertices;
		vertices->resize(idx);
		vertices->push_back((*vertices)[0]);

		if(i == 1)
			bounds = y_bounds;
	}
}

bool trivialReject(coordinate *cmd_parts, int length, int par_proj, int z_min, int z_proj)
{
	std::cerr << "Trivial Reject start\n";
	uint8_t codes[3];
	bounds x_bounds{ -1, 1 };
	bounds y_bounds{ -1, 1 };
	bounds z_bounds{ -1, 0 };
	if(!par_proj)
	{
		x_bounds.upper = -z_proj;
		x_bounds.lower = z_proj;
		y_bounds.upper = -z_proj;
		y_bounds.lower = z_proj;
		z_bounds.upper = z_min;
		z_bounds.lower = -1;
	}

	for(int i = 0; i < length; i += 2)
	{
		codes[i] = 0;
		if(cmd_parts[i].x < x_bounds.lower) codes[i] |= 0b000001;
		if(cmd_parts[i].x > x_bounds.upper) codes[i] |= 0b000010;
		if(cmd_parts[i].y < y_bounds.lower) codes[i] |= 0b000100;
		if(cmd_parts[i].y > y_bounds.upper) codes[i] |= 0b001000;
		if(cmd_parts[i].z < z_bounds.lower) codes[i] |= 0b010000;
		if(cmd_parts[i].z > z_bounds.upper) codes[i] |= 0b100000;
	}

	// Lie completely outside view window
	if((codes[0] & codes[1] & codes[2]) != 0b0000)
		return 0;

	// Lie completely inside view window
	return 1;
}

void checkPoint(coordinate *coord, bounds *x_bounds, bounds *y_bounds)
{
	if(coord->x > x_bounds->upper || coord->x < x_bounds->lower)
	{
		std::cerr << "ERROR: Coordinate is outside bounds " << x_bounds->lower << ", " << x_bounds->upper << '\n';
		std::cerr << "Coordinate is (" << coord->x << ", " << coord->y << ")\n";
	}

	if(coord->y > y_bounds->upper || coord->y < y_bounds->lower)
	{
		std::cerr << "ERROR: Coordinate is outside bounds " << x_bounds->lower << ", " << x_bounds->upper << '\n';
		std::cerr << "Coordinate is (" << coord->x << ", " << coord->y << ")\n";
	}
}

// Uses DDA algorithm to scan convert lines
void scanConversion(float *cmd_parts, std::vector<std::vector<uint8_t>> *pixels, bounds *x_bounds, bounds *y_bounds)
{
	std::cerr << "DDA Algorithm start\n";
	float x1{ cmd_parts[0] };
	float y1{ cmd_parts[1] };
	float x2{ cmd_parts[2] };
	float y2{ cmd_parts[3] };

	float dx{ x2 - x1 };
	float dy{ y2 - y1 };
	float m;
	if(abs((int)dx) >= abs((int)dy))
	{
		if(dx < 0)
		{
			std::swap(x1, x2);
			std::swap(y1, y2);
			dx = x2 - x1;
			dy = y2 - y1;
		}
		m = dy / dx;
		dx = 1;
	}
	else
	{
		if(dy < 0)
		{
			std::swap(x1, x2);
			std::swap(y1, y2);
			dx = x2 - x1;
			dy = y2 - y1;
		}
		m = dx / dy;
		dy = 1;
	}

	std::cerr << "m=" << m << " dx=" << dx << " dy=" << dy << "\n";
	if(dx == 1)
	{
		float y = y1;
		for(float x = x1; x < x2; ++x)
		{
			coordinate coord{ x, y, 0 };
			checkPoint(&coord, x_bounds, y_bounds);
			(*pixels)[round(y)][round(x)] = 1;
			y += m;
		}
	}
	else if(dy == 1)
	{
		float x = x1;
		for(float y = y1; y < y2; ++y)
		{
			coordinate coord{ x, y, 0 };
			checkPoint(&coord, x_bounds, y_bounds);
			(*pixels)[round(y)][round(x)] = 1;
			x += m;
		}
	}
}
