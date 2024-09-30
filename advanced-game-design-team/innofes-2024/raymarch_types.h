#ifndef RAYMARCH_TYPES_H
#define RAYMARCH_TYPES_H

#include "splashkit.h"
#include <cmath>
#include <vector>

// grid-march wants floats for screen dimensions, and wants a square area
const float SCREEN_WIDTH = 2560;
const float SCREEN_HEIGHT = 1440;//540 for the half-height demo
const int MARCH_RAYS = 360;
const double MAX_DISTANCE = std::sqrt((SCREEN_WIDTH * SCREEN_WIDTH) + (SCREEN_HEIGHT * SCREEN_HEIGHT));
const int NUM_OBSTACLES = 30;
const color RAY_COLOR = {1, 1, 1, 1};
//const color RAY_BG_COLOR = {1, 1, 1, 0.4};

struct ray {
    point_2d origin;
    vector_2d direction;
};

struct ray_hit {
    point_2d hit_point;
    double distance;
    bool hit_obstacle;
};

ray create_ray(point_2d origin, vector_2d direction);
std::vector<ray_hit> cast_rays(const std::vector<rectangle>& obstacles, point_2d origin, int num_rays, double max_distance);
void draw_rays(const std::vector<ray_hit>& hits, const point_2d& origin, color primary, color secondary);
void draw_obstacles(const std::vector<rectangle>& obstacles, color c);

#endif // RAYMARCH_TYPES_H
