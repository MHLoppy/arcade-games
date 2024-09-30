#include "raymarch_types.h"
#include "splashkit.h"
#include <algorithm>
#include <chrono>
#include <limits>

#include "gridmarch_types.h"
#include <algorithm>

Grid create_grid() {
    int cells_x = RES;
    int cells_y = RES;
    return {std::vector<Cell>(cells_x * cells_y, {CellType::Floor, 0.0}), cells_x, cells_y};
}

void set_cell(Grid& grid, int x, int y, CellType type) {
    if (x >= 0 && x < grid.width && y >= 0 && y < grid.height) {
        grid.cells[y * grid.width + x].type = type;
    }
}

Cell get_cell(const Grid& grid, int x, int y) {
    if (x >= 0 && x < grid.width && y >= 0 && y < grid.height) {
        return grid.cells[y * grid.width + x];
    }
    return {CellType::Wall, 0.0}; // Return wall for out-of-bounds
}

void march_light(Grid& grid, point_2d light_source, int num_rays, double max_distance) {
    // Reset luminance
    for (auto& cell : grid.cells) {
        cell.luminance = CELL_MIN_LUM;
    }

    for (int i = 0; i < num_rays; ++i) {
        double angle = i * (360.0 / num_rays);
        vector_2d direction = vector_from_angle(angle, 1.0);

        // Current position along the ray
        double current_x = light_source.x;
        double current_y = light_source.y;

        double total_distance = 0.0;

        while (total_distance <= max_distance) {
            int cell_x = static_cast<int>(current_x / CELL_SIZE);
            int cell_y = static_cast<int>(current_y / CELL_SIZE);

            // Calculate next cell in both X and Y directions
            int next_cell_x = static_cast<int>((current_x + direction.x * (CELL_SIZE / 2.0)) / CELL_SIZE);
            int next_cell_y = static_cast<int>((current_y + direction.y * (CELL_SIZE / 2.0)) / CELL_SIZE);

            bool hit_wall = false;

            // Check current and next cells for walls (to account for diagonal gaps)
            if (cell_x >= 0 && cell_x < grid.width && cell_y >= 0 && cell_y < grid.height) {
                Cell& current_cell = grid.cells[cell_y * grid.width + cell_x];

                // Update luminance
                double luminosity = std::max(0.0, 1.0 - total_distance / max_distance);
                if (luminosity > current_cell.luminance) {
                    current_cell.luminance = luminosity;
                }

                // Check for wall hit in current cell
                if (current_cell.type == CellType::Wall) {
                    hit_wall = true;
                }
            }

            // Check next cells for diagonal wall detection
            if (!hit_wall && next_cell_x >= 0 && next_cell_x < grid.width && cell_y >= 0 && cell_y < grid.height &&
                grid.cells[cell_y * grid.width + next_cell_x].type == CellType::Wall) {
                hit_wall = true;
            }
            if (!hit_wall && cell_x >= 0 && cell_x < grid.width && next_cell_y >= 0 && next_cell_y < grid.height &&
                grid.cells[next_cell_y * grid.width + cell_x].type == CellType::Wall) {
                hit_wall = true;
            }

            if (hit_wall || cell_x < 0 || cell_x >= grid.width || cell_y < 0 || cell_y >= grid.height) {
                // We've hit a wall or left the grid
                break;
            }

            // Move to next position
            current_x += direction.x * (CELL_SIZE / 2.0);
            current_y += direction.y * (CELL_SIZE / 2.0);
            total_distance += CELL_SIZE / 2.0;
        }
    }
}

const int WALL_MAX_LUM = 255;
const int AIR_MAX_LUM = 128;

void draw_grid(const Grid& grid) {
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const Cell& cell = get_cell(grid, x, y);
            color c;
            int luminance = static_cast<int>(cell.luminance * 255);

            if (cell.type == CellType::Wall) {
                c = rgba_color(luminance, luminance, luminance, WALL_MAX_LUM);
            } else {
                c = rgba_color(luminance, luminance, luminance, AIR_MAX_LUM);
            }
            fill_rectangle(c, x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE / 2, CELL_SIZE / 2);
        }
    }
}

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
    const float DEAD_ZONE_WIDTH  = SCREEN_WIDTH / 2;
    const float DEAD_ZONE_HEIGHT = SCREEN_HEIGHT / 2;

    const float DZ_X_MIN = (SCREEN_WIDTH - DEAD_ZONE_WIDTH) / 2;
    const float DZ_X_MAX = DZ_X_MIN + DEAD_ZONE_WIDTH;
    const float DZ_Y_MIN = (SCREEN_HEIGHT - DEAD_ZONE_HEIGHT) / 2;
    const float DZ_Y_MAX = DZ_Y_MIN + DEAD_ZONE_HEIGHT;

    const float BORDER_OFFSET = 30.0;

    bool in_dead_zone = (origin.x > DZ_X_MIN 
                         && origin.x < DZ_X_MAX 
                         && origin.y > DZ_Y_MIN 
                         && origin.y < DZ_Y_MAX);

    bool at_screen_edge = (origin.x < BORDER_OFFSET 
                           || origin.x > (SCREEN_WIDTH - BORDER_OFFSET) 
                           || origin.y < BORDER_OFFSET 
                           || origin.y > (SCREEN_HEIGHT - BORDER_OFFSET));

    const float X_INC = SCREEN_WIDTH * 0.0008 + 0.2;
    const float Y_INC = SCREEN_HEIGHT * 0.0008 + 0.2;

    // Move away from the center and extreme edges
    if (in_dead_zone)
    {
        if (origin.x < SCREEN_WIDTH / 2) origin.x -= X_INC;
        else origin.x += X_INC;
        if (origin.y < SCREEN_HEIGHT / 2) origin.y -= Y_INC;
        else origin.y += Y_INC;
    }
    else if (at_screen_edge)
    {
        if (origin.x < BORDER_OFFSET) origin.x += X_INC;
        else origin.x -= X_INC;
        if (origin.y < BORDER_OFFSET) origin.y += Y_INC;
        else origin.y -= Y_INC;
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

// Bresenham's line algorithm
void draw_line_on_grid(Grid& grid, int x1, int y1, int x2, int y2) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        set_cell(grid, x1, y1, CellType::Wall);

        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void setup_walls(Grid& grid) {
    // Create a few potentially-long random walls
    for (int i = 0; i < 3; ++i)
    {
        int x1 = rnd(grid.width);
        int y1 = rnd(grid.height);
        int x2 = rnd(grid.width);
        int y2 = rnd(grid.height);
        draw_line_on_grid(grid, x1, y1, x2, y2);
    }
    // Create some extra, shorter walls with more controlled distribution
    //   for reasons I haven't figured out, the "center" is north-west of the window's center
    float mid = 0.5;
    int x1 = rnd(grid.width * mid);
    int x2 = rnd(grid.width * mid);
    int y1 = rnd(grid.height * mid);
    int y2 = rnd(grid.height * mid);
    for (int i = 0; i < 8; ++i) // first two are top-left quadrant
    {
        if (i == 2)             // top-right quadrant
        {
            x1 = rnd((grid.width * mid), grid.width);
            x2 = rnd((grid.width * mid), grid.width);
        }
        else if (i == 4)        // bottom-right quadrant
        {
            y1 = rnd((grid.height * mid), grid.height);
            y2 = rnd((grid.height * mid), grid.height);
        }
        else if (i == 6)        // bottom-left quadrant
        {
            x1 = rnd(grid.width * mid);
            x2 = rnd(grid.width * mid);
        }

        int x_start = rnd(x1);
        int x_end = rnd(x2);
        int y_start = rnd(y1);
        int y_end = rnd(y2);
        draw_line_on_grid(grid, x_start, y_start, x_end, y_end);
    }

    // Create a border
    draw_line_on_grid(grid, 0, 0, grid.width - 1, 0);
    draw_line_on_grid(grid, 0, 0, 0, grid.height - 1);
    draw_line_on_grid(grid, grid.width - 1, 0, grid.width - 1, grid.height - 1);
    draw_line_on_grid(grid, 0, grid.height - 1, grid.width - 1, grid.height - 1);
}

int main() {
    const string NAME = "Tech Demo: Ray Marching";
    open_window(NAME, SCREEN_WIDTH, SCREEN_HEIGHT);
    window_toggle_border(NAME);
    window_toggle_fullscreen(NAME);

    // basic raymarch demo
    std::vector<rectangle> obstacles = generate_random_obstacles(NUM_OBSTACLES);

    // gridmarch
    Grid grid = create_grid();
    setup_walls(grid);

    // common
    bool grid_version = false;
    point_2d last_mouse_position = mouse_position();
    auto last_movement_time = std::chrono::steady_clock::now();

    while (not(quit_requested() || key_down(ESCAPE_KEY)))
    {
        process_events();
        clear_screen(COLOR_BLACK);

        // New: we check for activity
        point_2d current_mouse_position = mouse_position();
        auto current_time = std::chrono::steady_clock::now();

        if (key_typed(SPACE_KEY))
        {
            grid_version = !grid_version;
        }

        // Check if the mouse has moved
        if (current_mouse_position.x != last_mouse_position.x || current_mouse_position.y != last_mouse_position.y)
        {
            last_mouse_position = current_mouse_position;
            last_movement_time = current_time;
        }

        // Check if the mouse has moved in the last 10 seconds
        auto duration_since_last_movement = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_movement_time).count();
        point_2d origin;

        if (duration_since_last_movement >= 3)
        {
            walk_origin_around(origin);

            // the grid demo has a much less clear origin point, so highlight it
            if (grid_version)
            {
                circle grid_light_source = circle_at(origin, 10);
                draw_circle(COLOR_WHITE_SMOKE, grid_light_source);
                fill_circle(COLOR_GRAY, grid_light_source);
            }
        }

        else 
            origin = mouse_position();

        if (grid_version)
        {
            march_light(grid, origin, GRID_RAYS, MAX_LIGHT_DISTANCE);

            draw_grid(grid);
        }
        else
        {
            std::vector<ray_hit> hits = cast_rays(obstacles, origin, MARCH_RAYS, MAX_DISTANCE);

            draw_obstacles(obstacles, COLOR_DARK_GRAY);
            draw_rays(hits, origin, RAY_COLOR);
        }

        refresh_screen(60);
    }

    return 0;
}