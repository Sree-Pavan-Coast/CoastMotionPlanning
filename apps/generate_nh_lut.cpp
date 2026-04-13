/// Standalone CLI tool to precompute a dual-model non-holonomic heuristic LUT.
///
/// Usage:
///   ./generate_nh_lut --radius 8.0 --angle-bins 72 --grid-size 400 \
///                     --cell-size 0.1 --output configs/heuristics/nh_lut_dual.bin
///
/// This samples both Dubins and Reeds-Shepp distances with OMPL and writes
/// them into one binary LUT file.

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "coastmotionplanning/costs/dual_model_non_holonomic_heuristic.hpp"

using coastmotionplanning::costs::DualModelNonHolonomicHeuristic;
using coastmotionplanning::costs::HeuristicModel;

namespace {

struct Args {
    float radius = 8.0f;
    uint32_t angle_bins = 72;
    uint32_t grid_size = 400;
    float cell_size = 0.1f;
    std::string output = "nh_lut_dual.bin";
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
            std::cout
                << "Usage: generate_nh_lut [options]\n"
                << "  --radius <float>       Minimum turning radius in meters (default: 8.0)\n"
                << "  --angle-bins <int>     Number of heading bins (default: 72)\n"
                << "  --grid-size <int>      Grid dimension (default: 400)\n"
                << "  --cell-size <float>    Cell size in meters (default: 0.1)\n"
                << "  --output <path>        Output file path (default: nh_lut_dual.bin)\n";
            std::exit(0);
        }
    }
    return args;
}

} // namespace

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    std::cout << "=== Dual-Model Non-Holonomic Heuristic LUT Generator ===" << std::endl;
    std::cout << "  Turning radius: " << args.radius << " m" << std::endl;
    std::cout << "  Angle bins:     " << args.angle_bins << std::endl;
    std::cout << "  Grid size:      " << args.grid_size << " x " << args.grid_size << std::endl;
    std::cout << "  Cell size:      " << args.cell_size << " m" << std::endl;
    std::cout << "  Output:         " << args.output << std::endl;

    const auto start_time = std::chrono::high_resolution_clock::now();

    DualModelNonHolonomicHeuristic lut;
    lut.initGrid(args.radius, args.angle_bins, args.grid_size, args.cell_size);

    DualModelNonHolonomicHeuristic ompl_provider;
    ompl_provider.configureOmpl(args.radius);

    const int half_grid = static_cast<int>(args.grid_size) / 2;
    const int stored_y_size = half_grid + 1;
    const float bin_size = static_cast<float>(2.0 * M_PI / args.angle_bins);
    const size_t total_states =
        static_cast<size_t>(args.grid_size) * static_cast<size_t>(stored_y_size) * args.angle_bins;

    size_t completed = 0;
    const size_t report_interval = std::max<size_t>(1, total_states / 20);

    for (int x_idx = 0; x_idx < static_cast<int>(args.grid_size); ++x_idx) {
        const float dx = static_cast<float>(x_idx - half_grid) * args.cell_size;
        for (int y_idx = 0; y_idx < stored_y_size; ++y_idx) {
            const float dy = static_cast<float>(y_idx) * args.cell_size;
            for (uint32_t theta_idx = 0; theta_idx < args.angle_bins; ++theta_idx) {
                const float dtheta = static_cast<float>(theta_idx) * bin_size;

                lut.at(HeuristicModel::DUBINS, x_idx, y_idx, static_cast<int>(theta_idx)) =
                    ompl_provider.lookup(HeuristicModel::DUBINS, dx, dy, dtheta);
                lut.at(HeuristicModel::REEDS_SHEPP, x_idx, y_idx, static_cast<int>(theta_idx)) =
                    ompl_provider.lookup(HeuristicModel::REEDS_SHEPP, dx, dy, dtheta);

                ++completed;
                if (completed % report_interval == 0 || completed == total_states) {
                    const size_t percent = (completed * 100) / total_states;
                    std::cout << "  Progress: " << percent << "%" << std::endl;
                }
            }
        }
    }

    if (!lut.saveToFile(args.output)) {
        std::cerr << "ERROR: Failed to save LUT to: " << args.output << std::endl;
        return 1;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - start_time);
    std::cout << "LUT saved to: " << args.output << std::endl;
    std::cout << "Total generation time: " << elapsed.count() << " seconds" << std::endl;

    return 0;
}
