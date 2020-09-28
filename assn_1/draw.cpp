#include "draw.hpp"

// Apply scaling, rotation, and translation transformations
void applyTransformations(int *coords, transforms *transforms)
{
    // Translate scale coordinates
    if(transforms->scaling_factor != 1.0)
	for(int i = 0; i < 4; ++i)
	    coords[i] = coords[i] * transforms->scaling_factor;

    std::cerr << "Coordinates after scaling are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
    // Translate rotate coordinates CCW about origin
    if(transforms->rot_degree != 0)
    {
	for(int i = 0; i < 4; i += 2)
	{
	    float x = (float)coords[i];
	    float y = (float)coords[i+1];
	    float orig_angle = atan2(y, x);
	    float new_angle = orig_angle + transforms->rot_degree;
	    coords[i] = cos(new_angle);
            coords[i+1] = sin(new_angle);
	}
    }

    std::cerr << "Coordinates after rotation are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
    // Translate x coordinates
    if(transforms->x_translation != 0)
	for(int i = 0; i < 2; ++i)
	    coords[i] = coords[i] + transforms->x_translation;

    // Translate y coordinates
    if(transforms->y_translation != 0)
	for(int i = 2; i < 4; ++i)
	    coords[i] = coords[i] + transforms->y_translation;

    std::cerr << "Coordinates after translation are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
}

// Uses Cohen-Sutherland algorithm to clip lines to world view
void clipLine(int *cmd_parts)
{
    
}

// Uses Bresenham's algorithm to scan convert lines
void scanConversion(int *cmd_parts, std::vector<std::vector<uint8_t>> *pixels)
{
    int x1;
    int x2;
    if(cmd_parts[0] > cmd_parts[2])
    {
	x1 = cmd_parts[2];
	x2 = cmd_parts[0];
    }
    else
    {
	x1 = cmd_parts[0];
	x2 = cmd_parts[2];
    }

    int reflect_val = 1;
    if(cmd_parts[1] > cmd_parts[3])
	reflect_val = -1;
    
    int y1{ reflect_val * cmd_parts[1] };
    int y2{ reflect_val * cmd_parts[3] };
    int y{ y1 };
    
    int dx{ x2 - x1 };
    int dy{ y2 - y1 };
    int D { 2 * dy - dx };

    if(dx == 0 && dy != 0)
    {
	std::cerr << "Can't plot vertical line!";
    }
    else if(dx != 0 && dy == 0)
    {
	
    }
    else
    {
	for(int x = x1; x <= x2; ++x)
	{
	    
	}
    }
    
}
