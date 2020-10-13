#include "draw.hpp"

// Apply scaling, rotation, and translation transformations
void applyTransformations(float *coords, int length, transforms *transforms)
{
    // Translate scale coordinates
    if(transforms->scaling_factor != 1.0)
        for(int i = 0; i < length; ++i)
            coords[i] = coords[i] * transforms->scaling_factor;
    
    // Translate rotate coordinates CCW about origin
    if(transforms->rot_degree != 0)
    {
        double sin_deg = sin(transforms->rot_degree * M_PI / 180);
        double cos_deg = cos(transforms->rot_degree * M_PI / 180);
        for(int i = 0; i < length; i += 2)
        {
            float x = coords[i];
            float y = coords[i + 1];
            coords[i] = x * cos_deg - y * sin_deg;
            coords[i + 1] = x * sin_deg + y * cos_deg;
        }
    }

    // Translate x and y coordinates
    for(int i = 0; i < length; i += 2)
    {
        coords[i] = coords[i] + transforms->x_translation;
        coords[i + 1] = coords[i + 1] + transforms->y_translation;
    }
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
    }
    
    // Lie completely outside view window
    if((codes[0] & codes[1]) != 0b0000)
        return 0;

    // Lie completely inside view window
    return 1;
}
/*
// Clip the polygon with Sutherland-Hodgman
void clipPolygon(std::vector<coords> *vertices, bounds *x_bounds, bounds *y_bounds)
{
    std::vector<coords> new_vertices(vertices->size() * 2);
    float x1{ (*vertices)[0].x };
    int idx = 0;
    bounds *bounds = x_bounds;
    if(x1 >= bounds->lower)
    {
        new_vertices[idx] = (*vertices)[0];
        idx++;
    }

    // Clip against x lower bound
    for(int i = 0; i < vertices->size() - 1; ++i)
    {
        float x1{ (*vertices)[i].x };
        float y1{ (*vertices)[i].y };
        float x2{ (*vertices)[i + 1].x };
        float y2{ (*vertices)[i + 1].y };
        
        uint8_t v1_in = x1 >= bounds->lower;
        uint8_t v2_in = x2 >= bounds->lower;
        if(v1_in & v2_in)
        {
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(v1_in && !v2_in)
        {
            float xc{ (float)bounds->lower };
            y2 = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
            x2 = xc;
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(!v1_in && v2_in)
        {
            float xc{ (float)bounds->lower };
            y1 = (y1 - y2) * (xc - x2) / (x1 - x2) + y2;
            x1 = xc;
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
    idx = 0;
    x1 = (*vertices)[0].x;

    // Check first vertex
    if(x1 <= bounds->upper)
    {
        new_vertices[idx] = (*vertices)[0];
        idx++;
    }

    // Clip against x upper bound
    for(int i = 0; i < vertices->size() - 1; ++i)
    {
        float x1{ (*vertices)[i].x };
        float y1{ (*vertices)[i].y };
        float x2{ (*vertices)[i + 1].x };
        float y2{ (*vertices)[i + 1].y };
        
        uint8_t v1_in = x1 <= bounds->upper;
        uint8_t v2_in = x2 <= bounds->upper;
        if(v1_in & v2_in)
        {
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(v1_in && !v2_in)
        {
            float xc{ (float)bounds->upper };
            y2 = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
            x2 = xc;
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(!v1_in && v2_in)
        {
            float xc{ (float)bounds->upper };
            y1 = (y1 - y2) * (xc - x2) / (x1 - x2) + y2;
            x1 = xc;
            new_vertices[idx] = { x1, y1 };
            idx++;
                    
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
    }
    *vertices = new_vertices;
    vertices->resize(idx);
    vertices->push_back((*vertices)[0]);
    idx = 0;

    // Clip with y axis now
    bounds = y_bounds;
    float y1{ (*vertices)[0].y };

    // Check first vertex
    if(y1 >= bounds->lower)
    {
        new_vertices[idx] = (*vertices)[0];
        idx++;
    }
    
    // Clip against y lower bound
    for(int i = 0; i < vertices->size() - 1; ++i)
    {
        float x1{ (*vertices)[i].x };
        float y1{ (*vertices)[i].y };
        float x2{ (*vertices)[i + 1].x };
        float y2{ (*vertices)[i + 1].y };
        
        uint8_t v1_in = y1 >= bounds->lower;
        uint8_t v2_in = y2 >= bounds->lower;
        if(v1_in & v2_in)
        {
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(v1_in && !v2_in)
        {
            float yc{ (float)bounds->lower };
            x2 = (x2 - x1) * (yc - y1) / (y2 - y1)  + x1;
            y2 = yc;
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(!v1_in && v2_in)
        {
            float yc{ (float)bounds->lower };
            x1 = (x1 - x2) * (yc - y2) / (y1 - y2)  + x2;
            y1 = yc;
            new_vertices[idx] = { x1, y1 };
            idx++;
                    
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
    }
    *vertices = new_vertices;
    vertices->resize(idx);
    vertices->push_back((*vertices)[0]);
    idx = 0;
    y1 = (*vertices)[0].y;
    if(y1 <= bounds->upper)
    {
        new_vertices[idx] = (*vertices)[0];
        idx++;
    }
 
    // Clip against y upper bound
    for(int i = 0; i < vertices->size() - 1; ++i)
    {
        float x1{ (*vertices)[i].x };
        float y1{ (*vertices)[i].y };
        float x2{ (*vertices)[i + 1].x };
        float y2{ (*vertices)[i + 1].y };
        
        uint8_t v1_in = y1 <= bounds->upper;
        uint8_t v2_in = y2 <= bounds->upper;
        if(v1_in & v2_in)
        {
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(v1_in && !v2_in)
        {
            float yc{ (float)bounds->upper };
            x2 = (x2 - x1) * (yc - y1) / (y2 - y1)  + x1;
            y2 = yc;
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
        else if(!v1_in && v2_in)
        {
            float yc{ (float)bounds->upper };
            x1 = (x1 - x2) * (yc - y2) / (y1 - y2)  + x2;
            y1 = yc;
            new_vertices[idx] = { x1, y1 };
            idx++;
                    
            new_vertices[idx] = { x2, y2 };
            idx++;
        }
    }
    
    *vertices = new_vertices;
    vertices->resize(idx);
    vertices->push_back((*vertices)[0]);
}
*/
// Clip the polygon with Sutherland-Hodgman
void clipPolygon(std::vector<coords> *vertices, bounds *x_bounds, bounds *y_bounds)
{
    std::vector<coords> new_vertices(vertices->size() * 2);
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
                if(i < 2)
                {
                    y2 = (y2 - y1) * (xc - x1) / (x2 - x1) + y1;
                    x2 = xc;
                }
                else
                {
                    x2 = (x2 - x1) * (yc - y1) / (y2 - y1)  + x1;
                    y2 = yc;
                }
                new_vertices[idx] = { x2, y2 };
                idx++;
            }
            else if(!v1_in && v2_in)
            {
                if(i < 2)
                {
                    y1 = (y1 - y2) * (xc - x2) / (x1 - x2) + y2;
                    x1 = xc;
                }
                else
                {
                    x1 = (x1 - x2) * (yc - y2) / (y1 - y2)  + x2;
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
