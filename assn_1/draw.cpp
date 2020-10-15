#include "draw.h"

// Apply scaling, rotation, and translation transformations
void applyTransformations(float *coords, transforms *transforms)
{
    // Translate scale coordinates
    if(transforms->scaling_factor != 1.0)
        for(int i = 0; i < 4; ++i)
            coords[i] = coords[i] * transforms->scaling_factor;

    std::cerr << "Coordinates after scaling are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
    // Translate rotate coordinates CCW about origin
    if(transforms->rot_degree != 0)
    {
        double sin_deg = sin(transforms->rot_degree * M_PI / 180);
        double cos_deg = cos(transforms->rot_degree * M_PI / 180);
        for(int i = 0; i < 4; i += 2)
        {
            float x = coords[i];
            float y = coords[i + 1];
            coords[i] = x * cos_deg - y * sin_deg;
            coords[i + 1] = x * sin_deg + y * cos_deg;
        }
    }

    std::cerr << "Coordinates after rotation are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
    // Translate x and y coordinates
    for(int i = 0; i < 4; i += 2)
    {
        coords[i] = coords[i] + transforms->x_translation;
        coords[i + 1] = coords[i + 1] + transforms->y_translation;
    }

    std::cerr << "Coordinates after translation are x1=" << coords[0] << " y1=" << coords[1] << " x2=" << coords[2] << " y2=" << coords[3] << "\n";
    
}

// Uses Cohen-Sutherland algorithm to clip lines to world view, returns 1 if it should draw the line
int clipLine(float *cmd_parts, bounds *x_bounds, bounds *y_bounds)
{
    std::cerr << "Line Clipping Algorithm start\n";
    uint8_t codes[2] = { 0, 0 };

    for(int i = 0; i < 4; i += 2)
    {
        if(cmd_parts[i]     < x_bounds->lower) codes[i >> 1] |= 0b0001;
        if(cmd_parts[i]     > x_bounds->upper) codes[i >> 1] |= 0b0010;
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
        if(cmd_parts[point_idx]     < x_bounds->lower) codes[idx] |= 0b0001;
        if(cmd_parts[point_idx]     > x_bounds->upper) codes[idx] |= 0b0010;
        if(cmd_parts[point_idx + 1] < y_bounds->lower) codes[idx] |= 0b0100;
        if(cmd_parts[point_idx + 1] > y_bounds->upper) codes[idx] |= 0b1000;

        if(point_idx == 0) point_idx = 2;
        else point_idx = 0;
            //std::cerr << "(" << cmd_parts[0] << "," << cmd_parts[1] << ")(" << cmd_parts[2] << "," << cmd_parts[3] << ") Code: "
        //       << std::bitset<4>(codes[0]) << std::bitset<4>(codes[1]) << "\n";
    }
    
    // Lie completely outside view window
    if((codes[0] & codes[1]) != 0b0000)
        return 0;

    // Lie completely inside view window
    return 1;
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
