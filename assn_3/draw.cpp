#include "draw.h"

// Apply scaling, rotation, and translation transformations
void applyTransformations(float *coord, int length, arguments *args)
{
	for(int i  = 0; i < length; i += 2)
	{
		coordinate temp_coord{ coord[i], coord[i + 1] };
		// Scale coordinates
		float scaling_factors[2] { args->scaling_factor, args->scaling_factor };
		scaleCoord(&temp_coord, scaling_factors);
		
		// Rotate coordinates CCW about origin
		rotateCoord(&temp_coord, args->rot_deg);
		
		// Translate x and y coordinates
		translateCoord(&temp_coord, args->x_translation, args->y_translation);
		
		coord[i] = temp_coord.x;
		coord[i + 1] = temp_coord.y;
	}
}

void worldToViewport(coordinate *coord, int length, arguments *args)
{
	float x_scale = (float)(args->vw_x_upper_bound - args->vw_x_lower_bound) / (float)(args->ww_x_upper_bound - args->ww_x_lower_bound); 
	float y_scale = (float)(args->vw_y_upper_bound - args->vw_y_lower_bound) / (float)(args->ww_y_upper_bound - args->ww_y_lower_bound); 
	float scaling_factors[2] { x_scale, y_scale };
	// First translate to origin
	translateCoord(coord, -args->ww_x_lower_bound, -args->ww_y_lower_bound);
	
	// Second scale to viewport size
	scaleCoord(coord, scaling_factors);
	
	// Third translate to final position
	translateCoord(coord, args->vw_x_lower_bound, args->vw_y_lower_bound);
}
	
void scaleCoord(coordinate *coord, float *scaling_factors)
{
	if(scaling_factors[0] != 1.0)
		coord->x = coord->x * scaling_factors[0];

	if(scaling_factors[1] != 1.0)
		coord->y = coord->y * scaling_factors[1];
}

void rotateCoord(coordinate *coord, int rot_deg)
{
	if(rot_deg != 0)
	{
		double sin_deg = sin(rot_deg * M_PI / 180);
		double cos_deg = cos(rot_deg * M_PI / 180);
		float x = coord->x;
		float y = coord->y;
		coord->x = x * cos_deg - y * sin_deg;
		coord->y = x * sin_deg + y * cos_deg;
	}

}

void translateCoord(coordinate *coord, int x_trans, int y_trans)
{
	coord->x = coord->x + x_trans;
	coord->y = coord->y + y_trans;
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
			int xc = x_bounds->lower;
			cmd_parts[point_idx] = xc;
			cmd_parts[point_idx + 1] = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
		}
		else if((codes[idx] & 0b0010) == 0b0010)
		{
			int xc = x_bounds->upper;
			cmd_parts[point_idx] = xc;
			cmd_parts[point_idx + 1] = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
		}
		else if((codes[idx] & 0b0100) == 0b0100)
		{
			int yc = y_bounds->lower;
			cmd_parts[point_idx] = (x2 - x1) * (yc - y1) / (y2 - y1)  + x1;
			cmd_parts[point_idx + 1] = yc;
		}
		else if((codes[idx] & 0b1000) == 0b1000)
		{
			int yc = y_bounds->upper;
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

void fillPolygon(std::vector<std::vector<uint8_t>> *pixels, std::vector<coordinate> *vertices, bounds *x_bounds, bounds *y_bounds)
{
	std::cerr << "Polygon Fill Algorithm Started\n";
	std::vector<std::vector<int>> scan_lines;
	scan_lines.resize(y_bounds->upper);
	for(int i = 0; i < scan_lines.capacity(); ++i)
		scan_lines[i].reserve(vertices->size());

	int poly_y_min = (int)round((*vertices)[0].y);
	int poly_y_max = (int)round((*vertices)[0].y);
	for(int i = 0; i < vertices->size() - 1; ++i)
	{
		int x1{ (int)round((*vertices)[i].x) };
		int y1{ (int)round((*vertices)[i].y) };
		int x2{ (int)round((*vertices)[i + 1].x) };
		int y2{ (int)round((*vertices)[i + 1].y) };
		if(poly_y_max < y2)
			poly_y_max = y2;

		if(poly_y_min > y2)
			poly_y_min = y2;
		
		if (y1 > y2)
		{
			std::swap(y1, y2);
			std::swap(x1, x2);
		}
		
		for(int j = y1; j < y2; ++j)
		{
//			std::cerr << "j is " << j << " Vertices are (" << x1 << ", " << y1 << ") and (" << x2 << ", " << y2 << ")\n";
			if(y2 != y1)
			{
				int new_x = (int)round(x2 - (y2 - j) * (x2 - x1) / (float)(y2 - y1));
				scan_lines[j].push_back(new_x);
			}
		}
	}

	std::cerr << "Max poly y is " << poly_y_max << " Min poly y is " << poly_y_min << '\n';
	
	for(int i = poly_y_min; i < poly_y_max; i++)
	{
		if(scan_lines[i].empty())
			continue;

		std::vector<int> intersections { scan_lines[i] };
		std::sort(intersections.begin(), intersections.end());

		int intersect_idx = 0;
		uint8_t fill_val = 0;
		for(int j = x_bounds->lower; j < x_bounds->upper; ++j)
		{
			if(intersect_idx < intersections.size() && j >= intersections[intersect_idx])
			{
				(*pixels)[i][j] = 1;
				fill_val = ~fill_val;
				++intersect_idx;
			}

			if(fill_val != 0)
				(*pixels)[i][j] = 1;
		}
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
			(*pixels)[round(y - y_bounds->lower)][round(x - x_bounds->lower)] = 1;
			y += m;
		}
	}
	else if(dy == 1)
	{
		float x = x1;
		for(float y = y1; y < y2; ++y)
		{
			(*pixels)[round(y - y_bounds->lower)][round(x - x_bounds->lower)] = 1;
			x += m;
		}
	}
}
