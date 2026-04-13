#include "coastmotionplanning/costs/dual_model_non_holonomic_heuristic.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace coastmotionplanning {
namespace costs {

namespace {
namespace ob = ompl::base;

constexpr float TWO_PI = static_cast<float>(2.0 * M_PI);
constexpr float LARGE_VALUE = 1e6f;

float normalizeAngleSigned(float angle) {
    angle = std::fmod(angle + static_cast<float>(M_PI), TWO_PI);
    if (angle < 0.0f) {
        angle += TWO_PI;
    }
    return angle - static_cast<float>(M_PI);
}

ob::RealVectorBounds makeBounds(float min_turning_radius) {
    ob::RealVectorBounds bounds(2);
    const double extent = std::max<double>(1000.0, min_turning_radius * 1000.0);
    bounds.setLow(-extent);
    bounds.setHigh(extent);
    return bounds;
}

float computeOmplDistance(const ob::SE2StateSpace& space,
                          float dx,
                          float dy,
                          float dtheta) {
    ob::State* origin_state = space.allocState();
    ob::State* target_state = space.allocState();

    auto* origin = origin_state->as<ob::SE2StateSpace::StateType>();
    origin->setXY(0.0, 0.0);
    origin->setYaw(0.0);

    auto* target = target_state->as<ob::SE2StateSpace::StateType>();
    target->setXY(dx, dy);
    target->setYaw(normalizeAngleSigned(dtheta));

    const float distance = static_cast<float>(space.distance(origin_state, target_state));

    space.freeState(origin_state);
    space.freeState(target_state);
    return distance;
}

} // namespace

struct DualModelNonHolonomicHeuristic::OmplSpaces {
    std::unique_ptr<ob::DubinsStateSpace> dubins;
    std::unique_ptr<ob::ReedsSheppStateSpace> reeds_shepp;
};

DualModelNonHolonomicHeuristic::DualModelNonHolonomicHeuristic() = default;
DualModelNonHolonomicHeuristic::~DualModelNonHolonomicHeuristic() = default;
DualModelNonHolonomicHeuristic::DualModelNonHolonomicHeuristic(
    DualModelNonHolonomicHeuristic&&) noexcept = default;
DualModelNonHolonomicHeuristic& DualModelNonHolonomicHeuristic::operator=(
    DualModelNonHolonomicHeuristic&&) noexcept = default;

void DualModelNonHolonomicHeuristic::configureOmpl(float min_turning_radius) {
    header_.min_turning_radius = min_turning_radius;
    ompl_spaces_ = std::make_unique<OmplSpaces>();

    auto bounds = makeBounds(min_turning_radius);
    ompl_spaces_->dubins = std::make_unique<ob::DubinsStateSpace>(
        static_cast<double>(min_turning_radius), false);
    ompl_spaces_->dubins->setBounds(bounds);

    ompl_spaces_->reeds_shepp = std::make_unique<ob::ReedsSheppStateSpace>(
        static_cast<double>(min_turning_radius));
    ompl_spaces_->reeds_shepp->setBounds(bounds);
}

void DualModelNonHolonomicHeuristic::initGrid(float min_turning_radius,
                                              uint32_t num_angle_bins,
                                              uint32_t grid_size,
                                              float cell_size) {
    header_.magic = DualModelNHLutHeader::MAGIC;
    header_.version = DualModelNHLutHeader::VERSION;
    header_.min_turning_radius = min_turning_radius;
    header_.num_angle_bins = num_angle_bins;
    header_.grid_size = grid_size;
    header_.cell_size = cell_size;
    header_.stored_y_size = grid_size / 2 + 1;
    header_.model_count = 2;

    const size_t total =
        static_cast<size_t>(grid_size) * header_.stored_y_size * num_angle_bins;
    dubins_data_.assign(total, LARGE_VALUE);
    reeds_shepp_data_.assign(total, LARGE_VALUE);
    table_loaded_ = true;

    if (!ompl_spaces_ ||
        std::abs(header_.min_turning_radius - min_turning_radius) > 1e-6f) {
        configureOmpl(min_turning_radius);
    }
}

bool DualModelNonHolonomicHeuristic::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    DualModelNHLutHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(DualModelNHLutHeader));
    if (!file.good()) {
        return false;
    }

    if (header.magic != DualModelNHLutHeader::MAGIC ||
        header.version != DualModelNHLutHeader::VERSION ||
        header.model_count != 2 ||
        header.grid_size == 0 ||
        header.num_angle_bins == 0 ||
        header.cell_size <= 0.0f ||
        header.stored_y_size != header.grid_size / 2 + 1) {
        return false;
    }

    const size_t total = static_cast<size_t>(header.grid_size) *
                         header.stored_y_size * header.num_angle_bins;
    std::vector<float> dubins(total, LARGE_VALUE);
    std::vector<float> reeds_shepp(total, LARGE_VALUE);

    file.read(reinterpret_cast<char*>(dubins.data()),
              static_cast<std::streamsize>(total * sizeof(float)));
    file.read(reinterpret_cast<char*>(reeds_shepp.data()),
              static_cast<std::streamsize>(total * sizeof(float)));
    if (!file.good()) {
        return false;
    }

    header_ = header;
    dubins_data_ = std::move(dubins);
    reeds_shepp_data_ = std::move(reeds_shepp);
    table_loaded_ = true;
    configureOmpl(header_.min_turning_radius);
    return true;
}

bool DualModelNonHolonomicHeuristic::saveToFile(const std::string& filepath) const {
    if (dubins_data_.empty() || reeds_shepp_data_.empty() ||
        dubins_data_.size() != reeds_shepp_data_.size()) {
        return false;
    }

    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    file.write(reinterpret_cast<const char*>(&header_), sizeof(DualModelNHLutHeader));
    file.write(reinterpret_cast<const char*>(dubins_data_.data()),
               static_cast<std::streamsize>(dubins_data_.size() * sizeof(float)));
    file.write(reinterpret_cast<const char*>(reeds_shepp_data_.data()),
               static_cast<std::streamsize>(reeds_shepp_data_.size() * sizeof(float)));
    return file.good();
}

float DualModelNonHolonomicHeuristic::lookup(HeuristicModel model,
                                             float dx,
                                             float dy,
                                             float dtheta) const {
    if (table_loaded_) {
        float effective_dy = dy;
        float effective_dtheta = dtheta;
        if (effective_dy < 0.0f) {
            effective_dy = -effective_dy;
            effective_dtheta = -effective_dtheta;
        }

        int x_idx = 0;
        int y_idx = 0;
        int theta_idx = 0;
        if (toIndices(dx, effective_dy, effective_dtheta, x_idx, y_idx, theta_idx)) {
            return at(model, x_idx, y_idx, theta_idx);
        }
    }

    if (ompl_spaces_ == nullptr) {
        return LARGE_VALUE;
    }

    switch (model) {
        case HeuristicModel::DUBINS:
            return computeOmplDistance(*ompl_spaces_->dubins, dx, dy, dtheta);
        case HeuristicModel::REEDS_SHEPP:
            return computeOmplDistance(*ompl_spaces_->reeds_shepp, dx, dy, dtheta);
    }

    return LARGE_VALUE;
}

float& DualModelNonHolonomicHeuristic::at(HeuristicModel model,
                                          int x_idx,
                                          int y_idx,
                                          int theta_idx) {
    return dataFor(model).at(flatIndex(x_idx, y_idx, theta_idx));
}

float DualModelNonHolonomicHeuristic::at(HeuristicModel model,
                                         int x_idx,
                                         int y_idx,
                                         int theta_idx) const {
    return dataFor(model).at(flatIndex(x_idx, y_idx, theta_idx));
}

bool DualModelNonHolonomicHeuristic::toIndices(float dx,
                                               float dy,
                                               float dtheta,
                                               int& x_idx,
                                               int& y_idx,
                                               int& theta_idx) const {
    if (header_.grid_size == 0 || header_.num_angle_bins == 0 || header_.cell_size <= 0.0f) {
        return false;
    }

    const int half_grid = static_cast<int>(header_.grid_size) / 2;
    x_idx = static_cast<int>(std::lround(dx / header_.cell_size)) + half_grid;
    y_idx = static_cast<int>(std::lround(dy / header_.cell_size));

    const float bin_size = TWO_PI / static_cast<float>(header_.num_angle_bins);
    const float normalized_theta = normalizeAngle(dtheta);
    theta_idx = static_cast<int>(std::lround(normalized_theta / bin_size)) %
                static_cast<int>(header_.num_angle_bins);

    if (x_idx < 0 || x_idx >= static_cast<int>(header_.grid_size)) {
        return false;
    }
    if (y_idx < 0 || y_idx >= static_cast<int>(header_.stored_y_size)) {
        return false;
    }

    return true;
}

float DualModelNonHolonomicHeuristic::normalizeAngle(float angle) {
    angle = std::fmod(angle, TWO_PI);
    if (angle < 0.0f) {
        angle += TWO_PI;
    }
    return angle;
}

size_t DualModelNonHolonomicHeuristic::flatIndex(int x_idx,
                                                 int y_idx,
                                                 int theta_idx) const {
    if (x_idx < 0 || x_idx >= static_cast<int>(header_.grid_size) ||
        y_idx < 0 || y_idx >= static_cast<int>(header_.stored_y_size) ||
        theta_idx < 0 || theta_idx >= static_cast<int>(header_.num_angle_bins)) {
        throw std::out_of_range(
            "DualModelNonHolonomicHeuristic index is outside the lookup table bounds.");
    }

    return static_cast<size_t>(x_idx) * header_.stored_y_size * header_.num_angle_bins +
           static_cast<size_t>(y_idx) * header_.num_angle_bins +
           static_cast<size_t>(theta_idx);
}

std::vector<float>& DualModelNonHolonomicHeuristic::dataFor(HeuristicModel model) {
    return model == HeuristicModel::DUBINS ? dubins_data_ : reeds_shepp_data_;
}

const std::vector<float>& DualModelNonHolonomicHeuristic::dataFor(
    HeuristicModel model) const {
    return model == HeuristicModel::DUBINS ? dubins_data_ : reeds_shepp_data_;
}

} // namespace costs
} // namespace coastmotionplanning
