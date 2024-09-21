#include "raymarch_types.h"
#include "splashkit.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

ray create_ray(point_2d origin, vector_2d direction) {
    return {origin, unit_vector(direction)};
}

// Slab / Ray-AABB rect intsersection
bool ray_rectangle_intersection(const ray& r, const rectangle& rect, ray_hit& hit) {
    vector_2d ray_vector = vector_to(r.direction.x, r.direction.y);
    if (vector_in_rect(ray_vector, rect)) {
        hit.hit_point = r.origin;
        hit.distance = 0;
        hit.hit_obstacle = true;
        return true;
    }

    double t_near = -std::numeric_limits<double>::infinity();
    double t_far = std::numeric_limits<double>::infinity();

    vector_2d inv_dir = vector_to(1.0 / r.direction.x, 1.0 / r.direction.y);

    double t1x = (rect.x - r.origin.x) * inv_dir.x;
    double t2x = (rect.x + rect.width - r.origin.x) * inv_dir.x;
    double t1y = (rect.y - r.origin.y) * inv_dir.y;
    double t2y = (rect.y + rect.height - r.origin.y) * inv_dir.y;

    t_near = std::max(std::min(t1x, t2x), std::min(t1y, t2y));
    t_far = std::min(std::max(t1x, t2x), std::max(t1y, t2y));

    if (t_near > t_far || t_far < 0) return false;

    hit.distance = t_near;
    vector_2d hit_vector = vector_multiply(r.direction, t_near);
    hit.hit_point = point_at(r.origin.x + hit_vector.x, r.origin.y + hit_vector.y);
    hit.hit_obstacle = true;
    return true;
}

// not a real march, just using Ray-AABB intersection checks without marching through space
ray_hit march_ray(const ray& r, const std::vector<rectangle>& obstacles, double max_distance) {
    vector_2d end_vector = vector_multiply(r.direction, max_distance);
    point_2d end_point = point_at(r.origin.x + end_vector.x, r.origin.y + end_vector.y);
    ray_hit closest_hit = {end_point, max_distance, false};

    for (const auto& obs : obstacles) {
        ray_hit hit;
        if (ray_rectangle_intersection(r, obs, hit) && hit.distance < closest_hit.distance) {
            closest_hit = hit;
        }
    }

    return closest_hit;
}

std::vector<ray_hit> cast_rays(const std::vector<rectangle>& obstacles, point_2d origin, int num_rays, double max_distance) {
    std::vector<ray_hit> hits;
    hits.reserve(num_rays);

    for (int i = 0; i < num_rays; ++i) {
        double angle = i * (360.0 / num_rays);
        vector_2d direction = vector_from_angle(angle, 1.0);
        ray r = create_ray(origin, direction);
        hits.push_back(march_ray(r, obstacles, max_distance));
    }

    return hits;
}

void draw_rays(const std::vector<ray_hit>& hits, const point_2d& origin, color col)
{
    for (const auto& hit : hits)
    {
        draw_line(col, origin, hit.hit_point);
    }
}

void draw_obstacles(const std::vector<rectangle>& obstacles, color c) {
    for (const auto& obs : obstacles) {
        fill_rectangle(c, obs);
    }
}

const int SCREEN_WIDTH = 960;
const int SCREEN_HEIGHT = 540;
const int NUM_RAYS = 360;
const double MAX_DISTANCE = std::sqrt((SCREEN_WIDTH * SCREEN_WIDTH) + (SCREEN_HEIGHT * SCREEN_HEIGHT));
const int NUM_OBSTACLES = 30;
const color RAY_COLOR = {1, 1, 1, 1};
//const color RAY_BG_COLOR = {1, 1, 1, 0.4};

std::vector<rectangle> generate_random_obstacles(int num_obstacles) {
    std::vector<rectangle> obstacles;
    for (int i = 0; i < num_obstacles; ++i) {
        double x = rnd(SCREEN_WIDTH);
        double y = rnd(SCREEN_HEIGHT);
        double width = rnd(10, (SCREEN_WIDTH / 10 + 20));
        double height = rnd(10, 25);
        obstacles.push_back(rectangle_from(x, y, width, height));
    }
    return obstacles;
}

void walk_origin_around(point_2d &origin)
{
    // Calculate boundaries of dead zone (middle 1/9 of screen)
    const double DEAD_ZONE_WIDTH = SCREEN_WIDTH / 2;
    const double DEAD_ZONE_HEIGHT = SCREEN_HEIGHT / 2;
    const double DZ_X_MIN = (SCREEN_WIDTH - DEAD_ZONE_WIDTH) / 2;
    const double DZ_X_MAX = DZ_X_MIN + DEAD_ZONE_WIDTH;
    const double DZ_Y_MIN = (SCREEN_HEIGHT - DEAD_ZONE_HEIGHT) / 2;
    const double DZ_Y_MAX = DZ_Y_MIN + DEAD_ZONE_HEIGHT;

    bool in_dead_zone = (origin.x > DZ_X_MIN && origin.x < DZ_X_MAX 
                        && origin.y > DZ_Y_MIN && origin.y < DZ_Y_MAX);

    const float X_INC = 1.11;
    const float Y_INC = 0.77;

    if (in_dead_zone)
    {
        // Move the origin point away from the center
        if (origin.x < SCREEN_WIDTH / 2) origin.x -= X_INC;
        else origin.x += X_INC;
        if (origin.y < SCREEN_HEIGHT / 2) origin.y -= Y_INC;
        else origin.y += Y_INC;
    }
    else
    {
        // Otherwise walk clockwise between screen quadrants
        if (origin.x <= (SCREEN_WIDTH / 2) && origin.y <= (SCREEN_HEIGHT / 2)) origin.x += X_INC;     // top-left quadrant
        else if (origin.x > (SCREEN_WIDTH / 2) && origin.y <= (SCREEN_HEIGHT / 2)) origin.y += Y_INC; // top-right quadrant
        else if (origin.x > (SCREEN_WIDTH / 2) && origin.y > (SCREEN_HEIGHT / 2)) origin.x -= X_INC;  // bottom-right quadrant
        else if (origin.x <= (SCREEN_WIDTH / 2) && origin.y > (SCREEN_HEIGHT / 2)) origin.y -= Y_INC; // bottom-left quadrant
    }
}

int main() {
    open_window("Tech Demo: Simulated Ray Marching", SCREEN_WIDTH, SCREEN_HEIGHT);

    std::vector<rectangle> obstacles = generate_random_obstacles(NUM_OBSTACLES);

    bool inactive = true;
    unsigned int timer = 0;
    point_2d last_mouse_position = mouse_position();
    auto last_movement_time = std::chrono::steady_clock::now();

    while (not(quit_requested() || key_down(ESCAPE_KEY)))
    {
        process_events();
        clear_screen(COLOR_BLACK);

        // New: we check for activity
        point_2d current_mouse_position = mouse_position();
        auto current_time = std::chrono::steady_clock::now();

        // Check if the mouse has moved
        if (current_mouse_position.x != last_mouse_position.x || current_mouse_position.y != last_mouse_position.y)
        {
            last_mouse_position = current_mouse_position;
            last_movement_time = current_time;
        }

        // Check if the mouse has moved in the last 10 seconds
        auto duration_since_last_movement = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_movement_time).count();
        if (duration_since_last_movement >= 3)
            inactive = true;
        else
            inactive = false;

        point_2d origin;
        if (inactive)
        {
            walk_origin_around(origin);
        }
        else 
        {
            origin = mouse_position();
        }
        timer++;

        std::vector<ray_hit> hits = cast_rays(obstacles, origin, NUM_RAYS, MAX_DISTANCE);

        draw_obstacles(obstacles, COLOR_DARK_GRAY);
        draw_rays(hits, origin, RAY_COLOR);

        refresh_screen(60);
    }

    return 0;
}
