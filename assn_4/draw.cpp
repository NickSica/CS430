#include "draw.h"

// Quick and dirty dot product
void dotProduct(float *mat1, float *mat2, float *result, int m1_rows, int m1_cols, int m2_cols)
{
	for(int i = 0; i < m1_rows; ++i)
	{
		for (int j = 0; j < m2_cols; ++j)
		{
			float dot = 0;
			for (int k = 0; k < m1_cols; ++k)
				dot += *(mat1 + m1_cols * i + k) * *(mat2 + m2_cols * k + j);
			*(result + m2_cols * i + j) = dot;
		}
	}
}

void normalize(coordinate *coord, arguments *args)
{
	Eigen::Matrix<float, 4, 4> norm_mat;
	norm_mat <<
		1, 0, 0, -args->vrp_x,
		0, 1, 0, -args->vrp_y,
		0, 0, 1, -args->vrp_z,
		0, 0, 0, 1;

	Eigen::Matrix<float, 3, 1> rot_x;
	Eigen::Matrix<float, 3, 1> rot_y;
	Eigen::Matrix<float, 3, 1> rot_z;
	rotateViewplane(&rot_x, &rot_y, &rot_z, args);
	Eigen::Matrix<float, 4, 4> rot_mat;
	rot_mat <<
		rot_x[0], rot_x[1], rot_x[2], 0,
		rot_y[0], rot_y[1], rot_y[2], 0,
		rot_z[0], rot_z[1], rot_z[2], 0,
		0, 0, 0, 1;
	Eigen::Matrix<float, 4, 4> res_mat{ rot_mat * norm_mat };
	norm_mat = res_mat;

	float u_shear;
	float v_shear;
	if(args->prp_z != 0)
	{
		u_shear = (0.5 * (args->u_max + args->u_min) - args->prp_x) / args->prp_z;
		v_shear = (0.5 * (args->v_max + args->v_min) - args->prp_y) / args->prp_z;
	}
	else
	{
		u_shear = 0;
		v_shear = 0;
	}
	Eigen::Matrix<float, 4, 4> shear_mat;
	shear_mat <<
		1, 0, u_shear, 0,
		0, 1, v_shear, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	if(args->parallel_proj)
	{
		res_mat << shear_mat * norm_mat;
		norm_mat = res_mat;

		Eigen::Matrix<float, 4, 4> tran_mat;
		tran_mat <<
			1, 0, 0, -(args->u_max + args->u_min) / 2,
			0, 1, 0, -(args->v_max + args->v_min) / 2,
			0, 0, 1, -args->clip_front,
			0, 0, 0, 1;
		res_mat << tran_mat * norm_mat;
		norm_mat = res_mat;

		Eigen::Matrix<float, 4, 4> scale_mat;
		scale_mat <<
			2 / (args->u_max - args->u_min), 0, 0, 0,
			0, 2 / (args->v_max - args->v_min), 0, 0,
			0, 0, 1 / (args->clip_front - args->clip_back), 0,
			0, 0, 0, 1;
		res_mat << scale_mat * norm_mat;
		norm_mat = res_mat;
	}
	else
	{
		Eigen::Matrix<float, 4, 4> tran_mat;
		tran_mat <<
			1, 0, 0, -args->prp_x,
			0, 1, 0, -args->prp_y,
			0, 0, 1, -args->prp_z,
			0, 0, 0, 1;
		res_mat << tran_mat * norm_mat;
		norm_mat = res_mat;

		res_mat << shear_mat * norm_mat;
		norm_mat = res_mat;

		float u_diff = 1 / (args->u_max - args->u_min);
		float v_diff = 1 / (args->v_max - args->v_min);
		float prp_diff = args->prp_z - args->clip_back;
		if(prp_diff != 0)
			prp_diff = 1 / prp_diff;
		Eigen::Matrix<float, 4, 4> scale_mat;
		scale_mat <<
			2 * args->prp_z * u_diff * prp_diff, 0, 0, 0,
			0, 2 * args->prp_z * v_diff * prp_diff, 0, 0,
			0, 0, prp_diff, 0,
			0, 0, 0, 1;
		res_mat << scale_mat * norm_mat;
		norm_mat = res_mat;
	}

	Eigen::Matrix<float, 4, 1> old_coord;
	old_coord <<
		coord->x,
		coord->y,
		coord->z,
		1;
	Eigen::Matrix<float, 4, 1> new_coord{ norm_mat * old_coord };

	coord->x = new_coord[0];
	coord->y = new_coord[1];
	coord->z = new_coord[2];
}

void rotateViewplane(Eigen::Matrix<float, 3, 1> *rot_x, Eigen::Matrix<float, 3, 1> *rot_y, Eigen::Matrix<float, 3, 1> *rot_z, arguments *args)
{
	*rot_z <<
		args->vpn_x,
		args->vpn_y,
		args->vpn_z;
	rot_z->normalize();

	*rot_x <<
		args->vup_x,
		args->vup_y,
		args->vup_z;
	*rot_x = rot_x->cross(*rot_z);
	rot_x->normalize();

	*rot_y = rot_z->cross(*rot_x);
}

void worldToViewport(coordinate *coord, arguments *args, bounds *ww_x_bounds, bounds *ww_y_bounds)
{
	float x_scale = (float)(args->vw_x_upper_bound - args->vw_x_lower_bound) / (float)(ww_x_bounds->upper - ww_x_bounds->lower);
	float y_scale = (float)(args->vw_y_upper_bound - args->vw_y_lower_bound) / (float)(ww_y_bounds->upper - ww_y_bounds->lower);
	float scaling_factors[3] { x_scale, y_scale, 1.0 };
	// First translate to origin
	coordinate tran_coord{ -ww_x_bounds->lower, -ww_y_bounds->lower, 0 };
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

bool trivialReject(std::vector<coordinate> *face, int length, bool par_proj)
{
	std::cerr << "Trivial Reject start\n";
	uint8_t codes[face->size()];
	bounds xy_bounds{ -1, 1 };

	for(int i = 0; i < face->size(); ++i)
	{
		codes[i] = 0;
		if(!par_proj)
		{
			xy_bounds.lower = (*face)[i].z;
			xy_bounds.upper = -(*face)[i].z;
		}
		if((*face)[i].x < xy_bounds.lower) codes[i] |= 0b000001;
		if((*face)[i].x > xy_bounds.upper) codes[i] |= 0b000010;
		if((*face)[i].y < xy_bounds.lower) codes[i] |= 0b000100;
		if((*face)[i].y > xy_bounds.upper) codes[i] |= 0b001000;
	}

	uint8_t final_code{ 0b111111 };
	for(int i = 0; i < face->size(); ++i)
		final_code &= codes[i];

	// Lie completely outside view window
	if(final_code != 0b0000)
		return false;

	// Lie completely inside view window
	return true;
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
	//std::cerr << "Start: (" << x1 << ", " << y1 << ")\n";
	//std::cerr << "End: (" << x2 << ", " << y2 << ")\n";
	if(dx == 1)
	{
		float y = y1;
		for(float x = x1; x < x2; ++x)
		{
			coordinate coord{ x, y, 0 };
			checkPoint(&coord, x_bounds, y_bounds);
			//std::cerr << "Coordinate: " << round(x) << ", " << round(y) << '\n';
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
