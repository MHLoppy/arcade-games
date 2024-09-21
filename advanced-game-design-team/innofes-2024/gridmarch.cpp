#include "./gridmarch_types.h"
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
        cell.luminance = 0.0;
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

void draw_grid(const Grid& grid) {
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const Cell& cell = get_cell(grid, x, y);
            color c;
            int luminance = static_cast<int>(cell.luminance * 255);
            if (cell.type == CellType::Wall) {
                c = rgba_color(luminance, luminance, luminance, 255);
            } else {
                c = rgba_color(luminance, luminance, luminance, 100);
            }
            fill_rectangle(c, x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE / 2, CELL_SIZE / 2);
        }
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
    // Create random walls
    for (int i = 0; i < 7; ++i) {
        int x1 = rnd(grid.width);
        int y1 = rnd(grid.height);
        int x2 = rnd(grid.width);
        int y2 = rnd(grid.height);
        draw_line_on_grid(grid, x1, y1, x2, y2);
    }

    // Create a border
    draw_line_on_grid(grid, 0, 0, grid.width - 1, 0);
    draw_line_on_grid(grid, 0, 0, 0, grid.height - 1);
    draw_line_on_grid(grid, grid.width - 1, 0, grid.width - 1, grid.height - 1);
    draw_line_on_grid(grid, 0, grid.height - 1, grid.width - 1, grid.height - 1);
}

int main() {
    open_window("Grid-based Ray Marching Demo", SCREEN_WIDTH, SCREEN_HEIGHT);

    Grid grid = create_grid();
    setup_walls(grid);

    while (!quit_requested()) {
        process_events();
        clear_screen(COLOR_BLACK);

        point_2d mouse = mouse_position();
        march_light(grid, mouse, NUM_RAYS, MAX_LIGHT_DISTANCE);

        draw_grid(grid);

        refresh_screen(60);
    }

    return 0;
}