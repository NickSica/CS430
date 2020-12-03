#include "draw.h"

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
			0, 0, 1, -args->front_plane,
			0, 0, 0, 1;
		res_mat << tran_mat * norm_mat;
		norm_mat = res_mat;

		Eigen::Matrix<float, 4, 4> scale_mat;
		scale_mat <<
			2 / (args->u_max - args->u_min), 0, 0, 0,
			0, 2 / (args->v_max - args->v_min), 0, 0,
			0, 0, 1 / (args->front_plane - args->back_plane), 0,
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
		float prp_diff = args->prp_z - args->back_plane;
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

float interpolateZ(std::vector<coordinate> *vertices, scan_line *line_info, int x_p, int y_s)
{
	float y1{ line_info->edges[0].y };
	float y2{ line_info->edges[1].y };
	float y3{ line_info->edges[2].y };
	float y4{ line_info->edges[3].y };
	float z1{ line_info->edges[0].z };
	float z2{ line_info->edges[1].z };
	float z3{ line_info->edges[2].z };
	float z4{ line_info->edges[3].z };

	float x_a = line_info->intersections[0];
	float x_b = line_info->intersections[1];
	float z_a{ 0 };
	float z_b{ 0 };
	float z_p{ 0 };
	z_a = z1 - ((z1 - z2) * (y1 - y_s) / (y1 - y2));
	z_b = z3 - ((z3 - z4) * (y3 - y_s) / (y3 - y4));
	z_p = z_b - ((z_b - z_a) * (x_b - x_p) / (x_b - x_a));

	return z_p;
}

void updateZBuff(std::vector<std::vector<z_buffer>> *z_buff, uint8_t color[3], float front_plane, float z_p, int x, int y)
{
	if(z_p < front_plane && z_p > (*z_buff)[y][x].z)
	{
		(*z_buff)[y][x].z = z_p;
		(*z_buff)[y][x].color.red = color[0];
		(*z_buff)[y][x].color.green = color[1];
		(*z_buff)[y][x].color.blue = color[2];
	}
}

void fillTriangle(std::vector<std::vector<z_buffer>> *z_buff, std::vector<coordinate> *vertices,
				 uint8_t def_col[3], float front_plane, arguments *args, bounds *x_bounds, bounds *y_bounds)
{
	std::cerr << "Polygon Fill Algorithm Started\n";
	std::vector<scan_line> scan_lines;
	scan_lines.resize(y_bounds->upper);

	int poly_y_min = (int)std::round((*vertices)[0].y);
	int poly_y_max = (int)std::round((*vertices)[0].y);
	for(int i = 0; i < vertices->size() - 1; ++i)
	{
		int x1{ (int)std::round((*vertices)[i].x) };
		int y1{ (int)std::round((*vertices)[i].y) };
		int x2{ (int)std::round((*vertices)[i + 1].x) };
		int y2{ (int)std::round((*vertices)[i + 1].y) };

		float f_x1{ (*vertices)[i].x };
		float f_y1{ (*vertices)[i].y };
		float f_x2{ (*vertices)[i + 1].x };
		float f_y2{ (*vertices)[i + 1].y };
		if(poly_y_max < y2)
			poly_y_max = std::round(y2);

		if(poly_y_min > y2)
			poly_y_min = std::round(y2);

		if (y1 > y2)
		{
			std::swap(y1, y2);
			std::swap(x1, x2);
			std::swap(f_y1, f_y2);
			std::swap(f_x1, f_x2);
		}

		for(int j = y1; j < y2; ++j)
		{
			//std::cerr << "j is " << j << " Vertices are (" << x1 << ", " << y1 << ") and (" << x2 << ", " << y2 << ")\n";
			if(y2 != y1)
			{
				int new_x = (int)std::round(x2 - (y2 - j) * (x2 - x1) / (float)(y2 - y1));
				scan_lines[j].intersections.push_back(new_x);
				scan_lines[j].edges.push_back((*vertices)[i]);
				scan_lines[j].edges.push_back((*vertices)[i + 1]);
			}
		}
	}

	//std::cerr << "Max poly y is " << poly_y_max << " Min poly y is " << poly_y_min << '\n';
	uint8_t color[3];
	for(int i = 0; i < 3; ++i)
		color[i] = def_col[i];

	for(int i = poly_y_min; i < poly_y_max; ++i)
	{
		if(scan_lines[i].intersections.empty())
			continue;

		if(scan_lines[i].intersections[0] > scan_lines[i].intersections[1])
		{
			float temp = scan_lines[i].intersections[0];
			scan_lines[i].intersections[0] = scan_lines[i].intersections[1];
			scan_lines[i].intersections[1] = temp;

			coordinate temp_1 = scan_lines[i].edges[0];
			coordinate temp_2 = scan_lines[i].edges[1];
			scan_lines[i].edges[0] = scan_lines[i].edges[2];
			scan_lines[i].edges[1] = scan_lines[i].edges[3];
			scan_lines[i].edges[2] = temp_1;
			scan_lines[i].edges[3] = temp_2;
		}

		std::vector<int> intersections { scan_lines[i].intersections };
		if(intersections.size() > 2)
			std::cerr << "ERROR: More than 2 intersections in a triangle!";

				int intersect_idx = 0;
		uint8_t fill_val = 0;
		for(int j = x_bounds->lower; j < x_bounds->upper; ++j)
		{
			// Z interpolation and Depth Cueing
			float z_p = interpolateZ(vertices, &scan_lines[i], j, i);
			for(int i = 0; i < 3; i++)
				color[i] = (uint8_t)(std::round(def_col[i] * (z_p + 1) / (front_plane + 1)));
			if(intersect_idx < intersections.size() && j >= intersections[intersect_idx])
			{
				updateZBuff(z_buff, color, front_plane, z_p, j, i);
				/*
				if(j == 362 && i == 263)
				{

					for(int l = 0; l < vertices->size(); ++l)
						std::cerr << "ERROR: (" << (*vertices)[l].x << ", " << (*vertices)[l].y << ", " << (*vertices)[l].z << ")\n";
					std::cerr << "ITERATION " << i << " " << j << "\n";
				}
				*/
				fill_val = ~fill_val;
				++intersect_idx;
			}

			if(fill_val != 0)
			{
				updateZBuff(z_buff, color, front_plane, z_p, j, i);
				/*
				if(j == 362 && i == 263)
				{
					for(int l = 0; l < vertices->size(); ++l)
						std::cerr << "ERROR: (" << (*vertices)[l].x << ", " << (*vertices)[l].y << ", " << (*vertices)[l].z << ")\n";
					std::cerr << "ITERATION " << i << " " << j << "\n";
				}
				*/
			}
		}
	}
}
