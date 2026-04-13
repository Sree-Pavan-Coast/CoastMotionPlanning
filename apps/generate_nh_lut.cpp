/// Standalone CLI tool to precompute the non-holonomic heuristic lookup table.
///
/// Usage:
///   ./generate_nh_lut --radius 8.0 --angle-bins 72 --grid-size 400 \
///                     --cell-size 0.1 --output configs/heuristics/nh_lut_r8.0_a72.bin
///
/// This runs a 3D (x, y, theta) Dijkstra expansion using the Reeds-Shepp
/// kinematic model and writes the half-grid (y >= 0) binary LUT file.

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "coastmotionplanning/costs/non_holonomic_heuristic.hpp"

using namespace coastmotionplanning::costs;

namespace {

struct Args {
    float radius = 8.0f;
    uint32_t angle_bins = 72;
    uint32_t grid_size = 400;
    float cell_size = 0.1f;
    std::string output = "nh_lut.bin";
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--radius") == 0 && i + 1 < argc) {
            args.radius = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--angle-bins") == 0 && i + 1 < argc) {
            args.angle_bins = std::stoul(argv[++i]);
        } else if (std::strcmp(argv[i], "--grid-size") == 0 && i + 1 < argc) {
            args.grid_size = std::stoul(argv[++i]);
        } else if (std::strcmp(argv[i], "--cell-size") == 0 && i + 1 < argc) {
            args.cell_size = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            args.output = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: generate_nh_lut [options]\n"
                      << "  --radius <float>       Minimum turning radius in meters (default: 8.0)\n"
                      << "  --angle-bins <int>     Number of heading bins (default: 72)\n"
                      << "  --grid-size <int>      Grid dimension (default: 400)\n"
                      << "  --cell-size <float>    Cell size in meters (default: 0.1)\n"
                      << "  --output <path>        Output file path (default: nh_lut.bin)\n";
            std::exit(0);
        }
    }
    return args;
}

struct State3D {
    int x, y, theta;
    float cost;
    bool operator>(const State3D& other) const { return cost > other.cost; }
};

} // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    std::cout << "=== Non-Holonomic Heuristic LUT Generator ===" << std::endl;
    std::cout << "  Turning radius: " << args.radius << " m" << std::endl;
    std::cout << "  Angle bins:     " << args.angle_bins << std::endl;
    std::cout << "  Grid size:      " << args.grid_size << " x " << args.grid_size << std::endl;
    std::cout << "  Cell size:      " << args.cell_size << " m" << std::endl;
    std::cout << "  Output:         " << args.output << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize the LUT
    NonHolonomicHeuristic lut;
    lut.initGrid(args.radius, args.angle_bins, args.grid_size, args.cell_size);

    int half_grid = static_cast<int>(args.grid_size) / 2;
    int stored_y = half_grid + 1;  // y >= 0 only
    float bin_size = static_cast<float>(2.0 * M_PI / args.angle_bins);

    // The goal is at the center of the grid: (half_grid, 0, 0)
    // In the stored array, the goal is at x_idx=half_grid, y_idx=0, theta_idx=0

    // Dijkstra on 3D lattice
    // Each state is (x_idx, y_idx, theta_idx) where y_idx is in [0, stored_y)
    // For each heading, we expand forward and backward along arcs of minimum turning radius

    // Precompute motion primitives for one step
    // We generate: straight-forward, straight-back, left-forward, left-back, right-forward, right-back
    struct MotionDelta {
        float dx, dy, dtheta;  // in cell units and bin units
        float cost;            // arc length in cell units
    };

    std::vector<MotionDelta> primitives;

    // Step size: one cell length
    float step = 1.0f;  // in cell units
    float radius_cells = args.radius / args.cell_size;

    // For each heading bin, we precompute the motion deltas
    // Straight forward/backward
    // Turn left/right forward/backward

    // Arc length for turning: step = radius * dtheta => dtheta = step / radius
    float dtheta_turn = step / radius_cells;  // in radians
    float dtheta_bins = dtheta_turn / bin_size;  // in bin increments

    // Straight forward: dx=step in heading direction, dy=0, dtheta=0
    primitives.push_back({step, 0.0f, 0.0f, step * args.cell_size});
    // Straight backward
    primitives.push_back({-step, 0.0f, 0.0f, step * args.cell_size});

    // Left turn forward: arc of radius R, turning left (positive theta)
    float turn_dx = radius_cells * std::sin(dtheta_turn);
    float turn_dy = radius_cells * (1.0f - std::cos(dtheta_turn));
    float arc_len = radius_cells * dtheta_turn * args.cell_size;

    primitives.push_back({turn_dx, turn_dy, dtheta_bins, arc_len});    // Left forward
    primitives.push_back({turn_dx, -turn_dy, -dtheta_bins, arc_len});  // Right forward
    primitives.push_back({-turn_dx, turn_dy, -dtheta_bins, arc_len});  // Left backward
    primitives.push_back({-turn_dx, -turn_dy, dtheta_bins, arc_len});  // Right backward

    // 3D distance grid
    size_t total_cells = static_cast<size_t>(args.grid_size) * stored_y * args.angle_bins;
    std::vector<float> dist(total_cells, 1e9f);

    auto idx3d = [&](int x, int y, int t) -> size_t {
        return static_cast<size_t>(x) * stored_y * args.angle_bins +
               static_cast<size_t>(y) * args.angle_bins +
               static_cast<size_t>(t);
    };

    // Priority queue
    std::priority_queue<State3D, std::vector<State3D>, std::greater<State3D>> pq;

    // Seed: goal at center, heading 0
    // Since we compute heuristic as distance FROM any state TO goal,
    // we run Dijkstra BACKWARDS from the goal
    int goal_x = half_grid;
    int goal_y = 0;  // y=0 in the stored half
    for (uint32_t t = 0; t < args.angle_bins; ++t) {
        // We seed all theta bins at the goal since orientation shouldn't matter at the goal
        // Actually, we should seed only theta=0 and let the search find costs for other orientations
        // Let's seed all thetas at cost 0 at the goal position
        // This makes the heuristic: "how far to reach this (x,y) regardless of final orientation"
        // which is a valid admissible heuristic
    }
    // Seed only theta=0 for proper Reeds-Shepp semantics
    dist[idx3d(goal_x, goal_y, 0)] = 0.0f;
    pq.push({goal_x, goal_y, 0, 0.0f});

    size_t expanded = 0;
    size_t report_interval = total_cells / 10;

    std::cout << "Running 3D Dijkstra..." << std::endl;

    while (!pq.empty()) {
        State3D cur = pq.top();
        pq.pop();

        size_t cur_idx = idx3d(cur.x, cur.y, cur.theta);
        if (cur.cost > dist[cur_idx]) continue;

        expanded++;
        if (report_interval > 0 && expanded % report_interval == 0) {
            std::cout << "  Progress: " << (expanded * 100 / total_cells) << "%" << std::endl;
        }

        // Expand each primitive
        float heading_rad = cur.theta * bin_size;
        float cos_h = std::cos(heading_rad);
        float sin_h = std::sin(heading_rad);

        for (const auto& prim : primitives) {
            // Rotate primitive delta by current heading
            float rotated_dx = prim.dx * cos_h - prim.dy * sin_h;
            float rotated_dy = prim.dx * sin_h + prim.dy * cos_h;

            int nx = cur.x + static_cast<int>(std::round(rotated_dx));
            float ny_f = cur.y + rotated_dy;

            // Handle y symmetry: if ny goes negative, we can still store it
            // but we need to be careful. Since we only store y >= 0,
            // and the symmetry L(x,y,θ) = L(x,-y,-θ), we reflect:
            int ny = static_cast<int>(std::round(ny_f));
            int nt = (cur.theta + static_cast<int>(std::round(prim.dtheta)) + 
                     static_cast<int>(args.angle_bins)) % static_cast<int>(args.angle_bins);

            // If y < 0, apply symmetry
            if (ny < 0) {
                ny = -ny;
                nt = (static_cast<int>(args.angle_bins) - nt) % static_cast<int>(args.angle_bins);
            }

            // Bounds check
            if (nx < 0 || nx >= static_cast<int>(args.grid_size)) continue;
            if (ny >= stored_y) continue;

            float new_cost = cur.cost + prim.cost;
            size_t n_idx = idx3d(nx, ny, nt);

            if (new_cost < dist[n_idx]) {
                dist[n_idx] = new_cost;
                pq.push({nx, ny, nt, new_cost});
            }
        }
    }

    std::cout << "Dijkstra complete. Expanded " << expanded << " states." << std::endl;

    // Copy results into the LUT
    for (int x = 0; x < static_cast<int>(args.grid_size); ++x) {
        for (int y = 0; y < stored_y; ++y) {
            for (uint32_t t = 0; t < args.angle_bins; ++t) {
                lut.at(x, y, static_cast<int>(t)) = dist[idx3d(x, y, static_cast<int>(t))];
            }
        }
    }

    // Save
    if (lut.saveToFile(args.output)) {
        std::cout << "LUT saved to: " << args.output << std::endl;
    } else {
        std::cerr << "ERROR: Failed to save LUT to: " << args.output << std::endl;
        return 1;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - start_time);
    std::cout << "Total generation time: " << elapsed.count() << " seconds" << std::endl;

    return 0;
}
