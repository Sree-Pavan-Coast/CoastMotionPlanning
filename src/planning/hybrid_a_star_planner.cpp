#include "coastmotionplanning/planning/hybrid_a_star_planner.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include "coastmotionplanning/collision_checking/collision_checker.hpp"
#include "coastmotionplanning/common/math_constants.hpp"
#include "coastmotionplanning/costs/costmap_builder.hpp"
#include "coastmotionplanning/costs/costmap_types.hpp"
#include "coastmotionplanning/costs/holonomic_obstacles_heuristic.hpp"
#include "coastmotionplanning/costs/zone_selector.hpp"
#include "coastmotionplanning/motion_primitives/car_motion_table.hpp"
#include "coastmotionplanning/zones/maneuvering_zone.hpp"
#include "coastmotionplanning/zones/track_main_road.hpp"
#include "coastmotionplanning/zones/zone_type_utils.hpp"

namespace coastmotionplanning {
namespace planning {

namespace {

using Clock = std::chrono::steady_clock;
using collision_checking::CollisionChecker;
using collision_checking::CollisionCheckerConfig;
using costs::CostValues;
using costs::DualModelNonHolonomicHeuristic;
using costs::HeuristicModel;
using costs::ZoneSelectionResult;
using motion_primitives::CarMotionTable;
using motion_primitives::MotionTableConfig;
using motion_primitives::TurnDirection;
using MultiLineString2d = geometry::bg::model::multi_linestring<geometry::LineString2d>;
using MultiPolygon2d = geometry::bg::model::multi_polygon<geometry::Polygon2d>;

constexpr double LARGE_COST = 1e9;
constexpr double kTerminalStraightYawDeltaToleranceRad = 1e-3;
constexpr double kTerminalStraightAlignmentToleranceRad = 5e-2;
constexpr double kProjectionEpsilon = 1e-12;
constexpr double kOverlapAreaTolerance = 1e-9;
constexpr double kCandidateDedupTolerance = 1e-6;
constexpr double kInterfaceGuidanceTieBreakWeight = 0.01;
enum class TurnClass {
    UNKNOWN = 0,
    STRAIGHT,
    LEFT,
    RIGHT
};

enum class TransitionPromotionReasonCode {
    None = 0,
    AlignedAndDeepEnough,
    MaxDepthForced
};

struct SearchNode {
    math::Pose2d pose;
    size_t frontier_id{0};
    unsigned int heading_bin{0};
    double g{0.0};
    double h{0.0};
    int parent_index{-1};
    std::shared_ptr<zones::Zone> zone;
    std::string behavior_name;
    TurnClass last_turn_class{TurnClass::UNKNOWN};
    bool has_inbound_motion{false};
    common::MotionDirection inbound_motion{common::MotionDirection::Forward};
    double same_motion_length_m{0.0};
    double same_motion_remaining_to_change_m{0.0};
    ActiveZoneTransitionState active_transition;
    TransitionPromotionReasonCode transition_promotion_reason{
        TransitionPromotionReasonCode::None};
};

struct DiscreteStateKey {
    int x_idx{0};
    int y_idx{0};
    int theta_idx{0};
    int motion_state{0};
    int turn_class{0};
    int same_motion_length_bucket{0};
    int same_motion_remaining_bucket{0};
    int transition_state{0};
    int transition_entry_station_bucket{0};

    bool operator==(const DiscreteStateKey& other) const {
        return x_idx == other.x_idx &&
               y_idx == other.y_idx &&
               theta_idx == other.theta_idx &&
               motion_state == other.motion_state &&
               turn_class == other.turn_class &&
               same_motion_length_bucket == other.same_motion_length_bucket &&
               same_motion_remaining_bucket == other.same_motion_remaining_bucket &&
               transition_state == other.transition_state &&
               transition_entry_station_bucket == other.transition_entry_station_bucket;
    }
};

struct DiscreteStateKeyHash {
    size_t operator()(const DiscreteStateKey& key) const {
        size_t seed = std::hash<int>()(key.x_idx);
        seed ^= std::hash<int>()(key.y_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.theta_idx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.motion_state) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.turn_class) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^=
            std::hash<int>()(key.same_motion_length_bucket) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^=
            std::hash<int>()(key.same_motion_remaining_bucket) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.transition_state) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.transition_entry_station_bucket) + 0x9e3779b9 +
            (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct OpenEntry {
    double priority_f{0.0};
    double f{0.0};
    double h{0.0};
    double g{0.0};
    double guidance_hint{0.0};
    uint64_t insertion_order{0};
    size_t node_index{0};
    DiscreteStateKey state_key{};

    bool operator>(const OpenEntry& other) const {
        if (priority_f != other.priority_f) {
            return priority_f > other.priority_f;
        }
        if (f != other.f) {
            return f > other.f;
        }
        if (h != other.h) {
            return h > other.h;
        }
        return insertion_order > other.insertion_order;
    }
};

struct AnalyticPathValidationResult {
    bool valid{false};
    std::vector<common::MotionDirection> segment_directions;
};

struct SearchContext {
    std::string behavior_name;
    const PlannerBehaviorProfile* profile{nullptr};
    CarMotionTable motion_table;
    double xy_resolution_m{0.0};
    double yaw_bin_size_rad{0.0};
    double goal_distance_tolerance{0.0};
    double analytic_max_length_m{0.0};
    unsigned int analytic_expansion_interval{1};
    double analytic_step_size{0.0};
};

struct FrontierTrackLaneGuidanceRuntime {
    bool enabled{false};
    double target_station_m{0.0};
    costs::TrackLaneGuidanceTargetKind target_kind{
        costs::TrackLaneGuidanceTargetKind::Goal};
    double relax_window_m{0.0};
};

enum class FrontierGuidanceObjectiveKind {
    Goal = 0,
    Handoff
};

enum class FrontierHeuristicMode {
    GoalGrid = 0,
    InterfaceGrid,
    DirectPoseFallback
};

struct FrontierGuidanceContextRuntime {
    bool valid{false};
    FrontierGuidanceObjectiveKind objective_kind{FrontierGuidanceObjectiveKind::Goal};
    FrontierHeuristicMode heuristic_mode{FrontierHeuristicMode::GoalGrid};
    math::Pose2d representative_pose;
    std::vector<grid_map::Position> interface_seed_positions;
    grid_map::Matrix interface_heuristic_field;
    bool has_interface_heuristic_field{false};
};

struct TrackLaneRelaxationState {
    bool active{false};
    bool inside_relax_window{false};
    double bias_scale{1.0};
    double station_m{std::numeric_limits<double>::quiet_NaN()};
    double distance_to_target_m{std::numeric_limits<double>::infinity()};
};

struct PlannerPolylineSegment {
    geometry::Point2d start;
    geometry::Point2d end;
    double heading_rad{0.0};
    double length_sq{0.0};
    double length_m{0.0};
    double station_start_m{0.0};
};

struct PlannerPolylineProjection {
    bool valid{false};
    geometry::Point2d nearest_point;
    double distance_sq{std::numeric_limits<double>::infinity()};
    double station_m{0.0};
    double heading_rad{0.0};
};

struct FrontierCounters {
    uint64_t expansions_popped{0};
    uint64_t stale_entries_skipped{0};
    uint64_t goal_checks{0};
    uint64_t goal_hits{0};
    uint64_t lane_following_candidates{0};
    uint64_t lane_suppression_forward_only_applied{0};
    uint64_t lane_suppression_fallbacks{0};
    uint64_t analytic_attempts{0};
    uint64_t analytic_successes{0};
    uint64_t analytic_fail_no_ompl{0};
    uint64_t analytic_fail_same_motion_guard{0};
    uint64_t analytic_fail_path_length{0};
    uint64_t analytic_fail_out_of_bounds{0};
    uint64_t analytic_fail_collision{0};
    uint64_t analytic_fail_validation{0};
    uint64_t primitive_out_of_bounds{0};
    uint64_t primitive_cross_track_pruned{0};
    uint64_t primitive_behavior_unresolved{0};
    uint64_t primitive_disallowed{0};
    uint64_t primitive_collision{0};
    uint64_t primitive_motion_change_blocked{0};
    uint64_t primitive_dominated{0};
    uint64_t primitive_enqueued{0};
};

struct FrontierRuntime {
    costs::SearchFrontierDescriptor descriptor;
    const SearchContext* context{nullptr};
    mutable std::mutex mutex;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_queue;
    std::unordered_map<DiscreteStateKey, double, DiscreteStateKeyHash> best_g_by_key;
    common::ProfilingCollector profiling_collector;
    FrontierCounters counters;
    PlannerFrontierDebugSummary debug_summary;
    std::vector<PlannerExpansionDebugEvent> expansion_events;
};

struct SharedPlannerResult {
    HybridAStarPlannerResult result;
    bool published{false};
};

struct GoalCandidate {
    HybridAStarPlannerResult result;
    double terminal_straight_approach_m{0.0};
    double required_straight_approach_m{0.0};
    double path_cost{std::numeric_limits<double>::infinity()};
    bool valid{false};
};

bool canChangeMotionDirection(const SearchNode& node,
                              common::MotionDirection successor_motion);
double computeNextSameMotionLength(const SearchNode& parent,
                                   common::MotionDirection successor_motion,
                                   double travel_m,
                                   double global_max_same_motion_length_m);
double computeNextSameMotionRemainingToChange(
    const SearchNode& parent,
    const PlannerBehaviorProfile& current_profile,
    const PlannerBehaviorProfile& successor_profile,
    common::MotionDirection successor_motion,
    double travel_m);
bool isTerminalMotionSegmentValid(const SearchNode& node);
double computeNonHolonomicHeuristic(const DualModelNonHolonomicHeuristic& heuristic,
                                    HeuristicModel model,
                                    const math::Pose2d& current_pose,
                                    const math::Pose2d& target_pose,
                                    common::ProfilingCollector* profiler = nullptr);

double normalizeAngleSigned(double angle) {
    angle = std::fmod(angle + common::PI, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle - common::PI;
}

double normalizeAngleUnsigned(double angle) {
    angle = std::fmod(angle, common::TWO_PI);
    if (angle < 0.0) {
        angle += common::TWO_PI;
    }
    return angle;
}

double squaredDistance(const geometry::Point2d& first, const geometry::Point2d& second) {
    const double dx = first.x() - second.x();
    const double dy = first.y() - second.y();
    return (dx * dx) + (dy * dy);
}

std::vector<PlannerPolylineSegment> buildPlannerPolylineSegments(
    const std::vector<math::Pose2d>& polyline) {
    std::vector<PlannerPolylineSegment> segments;
    double station = 0.0;
    for (size_t idx = 0; idx + 1 < polyline.size(); ++idx) {
        const geometry::Point2d start(polyline[idx].x, polyline[idx].y);
        const geometry::Point2d end(polyline[idx + 1].x, polyline[idx + 1].y);
        const double dx = end.x() - start.x();
        const double dy = end.y() - start.y();
        const double length_sq = (dx * dx) + (dy * dy);
        if (length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double length_m = std::sqrt(length_sq);
        PlannerPolylineSegment segment;
        segment.start = start;
        segment.end = end;
        segment.heading_rad = std::atan2(dy, dx);
        segment.length_sq = length_sq;
        segment.length_m = length_m;
        segment.station_start_m = station;
        segments.push_back(segment);
        station += length_m;
    }
    return segments;
}

double plannerPolylineLength(const std::vector<PlannerPolylineSegment>& segments) {
    if (segments.empty()) {
        return 0.0;
    }
    const auto& last = segments.back();
    return last.station_start_m + last.length_m;
}

PlannerPolylineProjection projectOntoPlannerSegments(
    const geometry::Point2d& point,
    const std::vector<PlannerPolylineSegment>& segments) {
    PlannerPolylineProjection best;
    for (const auto& segment : segments) {
        if (segment.length_sq <= kProjectionEpsilon) {
            continue;
        }

        const double px = point.x();
        const double py = point.y();
        const double x0 = segment.start.x();
        const double y0 = segment.start.y();
        const double x1 = segment.end.x();
        const double y1 = segment.end.y();
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double t = std::clamp(
            (((px - x0) * dx) + ((py - y0) * dy)) / segment.length_sq,
            0.0,
            1.0);
        const geometry::Point2d nearest_point(x0 + (t * dx), y0 + (t * dy));
        const double distance_sq = squaredDistance(point, nearest_point);
        if (best.valid && distance_sq >= best.distance_sq) {
            continue;
        }

        best.valid = true;
        best.nearest_point = nearest_point;
        best.distance_sq = distance_sq;
        best.station_m = segment.station_start_m + (t * segment.length_m);
        best.heading_rad = segment.heading_rad;
    }
    return best;
}

std::optional<math::Pose2d> interpolatePoseOnPlannerSegments(
    const std::vector<PlannerPolylineSegment>& segments,
    double station_m) {
    if (segments.empty()) {
        return std::nullopt;
    }

    const double total_length = plannerPolylineLength(segments);
    const double clamped_station = std::clamp(station_m, 0.0, total_length);
    for (const auto& segment : segments) {
        const double segment_end = segment.station_start_m + segment.length_m;
        if (clamped_station > segment_end + kProjectionEpsilon) {
            continue;
        }

        const double local_station = clamped_station - segment.station_start_m;
        const double t = segment.length_m > kProjectionEpsilon
            ? std::clamp(local_station / segment.length_m, 0.0, 1.0)
            : 0.0;
        return math::Pose2d(
            segment.start.x() + ((segment.end.x() - segment.start.x()) * t),
            segment.start.y() + ((segment.end.y() - segment.start.y()) * t),
            math::Angle::from_radians(segment.heading_rad));
    }

    const auto& last = segments.back();
    return math::Pose2d(
        last.end.x(),
        last.end.y(),
        math::Angle::from_radians(last.heading_rad));
}

geometry::Polygon2d correctedPolygon(geometry::Polygon2d polygon) {
    geometry::bg::correct(polygon);
    return polygon;
}

geometry::LineString2d boundaryLineString(const geometry::Polygon2d& polygon) {
    geometry::LineString2d boundary;
    const auto& outer = polygon.outer();
    if (outer.empty()) {
        return boundary;
    }

    for (const auto& point : outer) {
        boundary.push_back(point);
    }
    if (!geometry::arePointsClose(boundary.front(), boundary.back())) {
        boundary.push_back(boundary.front());
    }
    return boundary;
}

void appendUniqueCandidate(std::vector<geometry::Point2d>& candidates,
                           const geometry::Point2d& candidate) {
    const bool duplicate = std::any_of(
        candidates.begin(),
        candidates.end(),
        [&](const auto& existing) {
            return geometry::arePointsClose(existing, candidate, kCandidateDedupTolerance);
        });
    if (!duplicate) {
        candidates.push_back(candidate);
    }
}

void appendLineCandidates(std::vector<geometry::Point2d>& candidates,
                          const geometry::LineString2d& line) {
    if (line.empty()) {
        return;
    }

    for (size_t idx = 0; idx < line.size(); ++idx) {
        appendUniqueCandidate(candidates, line[idx]);
        if (idx + 1 >= line.size()) {
            continue;
        }
        if (geometry::arePointsClose(line[idx], line[idx + 1], kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (line[idx].x() + line[idx + 1].x()) * 0.5,
                (line[idx].y() + line[idx + 1].y()) * 0.5));
    }
}

void appendPolygonCandidates(std::vector<geometry::Point2d>& candidates,
                             const geometry::Polygon2d& polygon) {
    const auto& outer = polygon.outer();
    if (outer.size() < 2) {
        return;
    }

    const size_t unique_vertex_count =
        geometry::arePointsClose(outer.front(), outer.back(), kCandidateDedupTolerance)
            ? outer.size() - 1
            : outer.size();
    for (size_t idx = 0; idx < unique_vertex_count; ++idx) {
        appendUniqueCandidate(candidates, outer[idx]);
        const auto& next = outer[(idx + 1) % unique_vertex_count];
        if (geometry::arePointsClose(outer[idx], next, kCandidateDedupTolerance)) {
            continue;
        }
        appendUniqueCandidate(
            candidates,
            geometry::Point2d(
                (outer[idx].x() + next.x()) * 0.5,
                (outer[idx].y() + next.y()) * 0.5));
    }
}

void appendClosestBridgeCandidates(std::vector<geometry::Point2d>& candidates,
                                   const geometry::Polygon2d& current_polygon,
                                   const geometry::Polygon2d& next_polygon) {
    std::vector<geometry::Point2d> current_samples;
    std::vector<geometry::Point2d> next_samples;
    appendPolygonCandidates(current_samples, correctedPolygon(current_polygon));
    appendPolygonCandidates(next_samples, correctedPolygon(next_polygon));
    if (current_samples.empty() || next_samples.empty()) {
        return;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    geometry::Point2d best_current;
    geometry::Point2d best_next;
    for (const auto& current_sample : current_samples) {
        for (const auto& next_sample : next_samples) {
            const double distance_sq = squaredDistance(current_sample, next_sample);
            if (distance_sq >= best_distance_sq) {
                continue;
            }
            best_distance_sq = distance_sq;
            best_current = current_sample;
            best_next = next_sample;
        }
    }

    if (!std::isfinite(best_distance_sq)) {
        return;
    }

    appendUniqueCandidate(candidates, best_current);
    appendUniqueCandidate(
        candidates,
        geometry::Point2d(
            (best_current.x() + best_next.x()) * 0.5,
            (best_current.y() + best_next.y()) * 0.5));
    appendUniqueCandidate(candidates, best_next);
}

std::vector<geometry::Point2d> deriveHandoffCandidates(
    const geometry::Polygon2d& current_polygon,
    const geometry::Polygon2d& next_polygon) {
    std::vector<geometry::Point2d> candidates;

    MultiPolygon2d overlap;
    geometry::bg::intersection(
        correctedPolygon(current_polygon),
        correctedPolygon(next_polygon),
        overlap);

    double overlap_area = 0.0;
    for (const auto& polygon : overlap) {
        overlap_area += std::abs(geometry::bg::area(polygon));
    }
    if (overlap_area > kOverlapAreaTolerance) {
        for (const auto& polygon : overlap) {
            appendPolygonCandidates(candidates, correctedPolygon(polygon));
        }
        return candidates;
    }

    MultiLineString2d shared_boundary;
    geometry::bg::intersection(
        boundaryLineString(correctedPolygon(current_polygon)),
        boundaryLineString(correctedPolygon(next_polygon)),
        shared_boundary);
    for (const auto& line : shared_boundary) {
        appendLineCandidates(candidates, line);
    }

    if (candidates.empty()) {
        appendClosestBridgeCandidates(candidates, current_polygon, next_polygon);
    }

    return candidates;
}

std::vector<grid_map::Position> selectInterfaceSeedPositions(
    const costs::SearchFrontierDescriptor& current_frontier,
    const costs::SearchFrontierDescriptor& next_frontier,
    const math::Pose2d& score_pose,
    size_t max_seed_count = 8) {
    if (current_frontier.zone == nullptr || next_frontier.zone == nullptr) {
        return {};
    }

    auto candidates = deriveHandoffCandidates(
        current_frontier.zone->getPolygon(),
        next_frontier.zone->getPolygon());
    if (candidates.empty()) {
        return {};
    }

    const geometry::Point2d score_point(score_pose.x, score_pose.y);
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&](const auto& lhs, const auto& rhs) {
            return squaredDistance(lhs, score_point) < squaredDistance(rhs, score_point);
        });

    std::vector<grid_map::Position> seeds;
    seeds.reserve(std::min(max_seed_count, candidates.size()));
    for (size_t idx = 0; idx < candidates.size() && seeds.size() < max_seed_count; ++idx) {
        seeds.emplace_back(candidates[idx].x(), candidates[idx].y());
    }
    return seeds;
}

bool directGoalSegmentHitsHandoffRegion(const math::Pose2d& start_pose,
                                        const math::Pose2d& goal_pose,
                                        const geometry::Polygon2d& current_polygon,
                                        const geometry::Polygon2d& next_polygon) {
    geometry::LineString2d direct_segment;
    direct_segment.push_back(geometry::Point2d(start_pose.x, start_pose.y));
    direct_segment.push_back(geometry::Point2d(goal_pose.x, goal_pose.y));

    MultiPolygon2d overlap;
    geometry::bg::intersection(
        correctedPolygon(current_polygon),
        correctedPolygon(next_polygon),
        overlap);
    double overlap_area = 0.0;
    for (const auto& polygon : overlap) {
        overlap_area += std::abs(geometry::bg::area(polygon));
    }
    if (overlap_area > kOverlapAreaTolerance) {
        for (const auto& polygon : overlap) {
            if (geometry::bg::intersects(direct_segment, correctedPolygon(polygon))) {
                return true;
            }
        }
    }

    MultiLineString2d shared_boundary;
    geometry::bg::intersection(
        boundaryLineString(correctedPolygon(current_polygon)),
        boundaryLineString(correctedPolygon(next_polygon)),
        shared_boundary);
    for (const auto& line : shared_boundary) {
        if (geometry::bg::intersects(direct_segment, line)) {
            return true;
        }
    }

    return false;
}

std::optional<math::Pose2d> computeGoalDirectedTrackEntryPose(
    const std::shared_ptr<zones::Zone>& source_zone,
    const std::shared_ptr<zones::TrackMainRoad>& track,
    const math::Pose2d& goal_pose,
    double target_station_m,
    double min_depth_m) {
    if (source_zone == nullptr || track == nullptr) {
        return std::nullopt;
    }

    const auto centerline_segments = buildPlannerPolylineSegments(track->getCenterline());
    const auto& lanes = track->getLanes();
    if (centerline_segments.empty() || lanes.size() < 2) {
        return std::nullopt;
    }

    const auto lane0_segments = buildPlannerPolylineSegments(lanes[0]);
    const auto lane1_segments = buildPlannerPolylineSegments(lanes[1]);
    if (lane0_segments.empty() || lane1_segments.empty()) {
        return std::nullopt;
    }

    const auto candidates = deriveHandoffCandidates(
        source_zone->getPolygon(),
        track->getPolygon());
    if (candidates.empty()) {
        return std::nullopt;
    }

    const geometry::Point2d goal_point(goal_pose.x, goal_pose.y);
    PlannerPolylineProjection entry_projection;
    for (const auto& candidate : candidates) {
        const auto projection = projectOntoPlannerSegments(candidate, centerline_segments);
        if (!projection.valid) {
            continue;
        }
        if (!entry_projection.valid ||
            squaredDistance(candidate, goal_point) <
                squaredDistance(entry_projection.nearest_point, goal_point)) {
            entry_projection = projection;
        }
    }
    if (!entry_projection.valid) {
        return std::nullopt;
    }

    const double total_centerline_length = plannerPolylineLength(centerline_segments);
    const size_t preferred_lane_idx =
        target_station_m >= entry_projection.station_m ? 0u : 1u;
    const double clamped_depth = std::max(min_depth_m, 0.0);
    const double desired_centerline_station =
        preferred_lane_idx == 0u
            ? std::min(total_centerline_length, entry_projection.station_m + clamped_depth)
            : std::max(0.0, entry_projection.station_m - clamped_depth);
    if (std::abs(desired_centerline_station - entry_projection.station_m) <= kProjectionEpsilon) {
        return std::nullopt;
    }

    const double desired_lane_station =
        preferred_lane_idx == 0u
            ? desired_centerline_station
            : total_centerline_length - desired_centerline_station;
    const auto snapped_pose = interpolatePoseOnPlannerSegments(
        preferred_lane_idx == 0u ? lane0_segments : lane1_segments,
        desired_lane_station);
    if (!snapped_pose.has_value()) {
        return std::nullopt;
    }
    if (!costs::ZoneSelector::isInsidePolygon(
            geometry::Point2d(snapped_pose->x, snapped_pose->y),
            track->getPolygon())) {
        return std::nullopt;
    }
    return snapped_pose;
}

TurnClass classifyTurn(TurnDirection turn_direction) {
    switch (turn_direction) {
        case TurnDirection::FORWARD:
        case TurnDirection::REVERSE:
            return TurnClass::STRAIGHT;
        case TurnDirection::LEFT:
        case TurnDirection::REV_LEFT:
            return TurnClass::LEFT;
        case TurnDirection::RIGHT:
        case TurnDirection::REV_RIGHT:
            return TurnClass::RIGHT;
        case TurnDirection::UNKNOWN:
            return TurnClass::UNKNOWN;
    }
    return TurnClass::UNKNOWN;
}

common::MotionDirection motionDirectionForPrimitive(TurnDirection turn_direction) {
    switch (turn_direction) {
        case TurnDirection::REVERSE:
        case TurnDirection::REV_LEFT:
        case TurnDirection::REV_RIGHT:
            return common::MotionDirection::Reverse;
        case TurnDirection::FORWARD:
        case TurnDirection::LEFT:
        case TurnDirection::RIGHT:
        case TurnDirection::UNKNOWN:
            return common::MotionDirection::Forward;
    }
    return common::MotionDirection::Forward;
}

DiscreteStateKey discretizeState(const math::Pose2d& pose,
                                 unsigned int heading_bin,
                                 const SearchNode& node,
                                 double xy_resolution_m,
                                 double global_max_same_motion_length_m) {
    const auto discretizeMetric = [&](double value) {
        if (global_max_same_motion_length_m <= common::EPSILON ||
            xy_resolution_m <= common::EPSILON) {
            return 0;
        }
        const double capped = std::clamp(value, 0.0, global_max_same_motion_length_m);
        return static_cast<int>(std::lround(capped / xy_resolution_m));
    };

    return DiscreteStateKey{
        static_cast<int>(std::lround(pose.x / xy_resolution_m)),
        static_cast<int>(std::lround(pose.y / xy_resolution_m)),
        static_cast<int>(heading_bin),
        node.has_inbound_motion
            ? (node.inbound_motion == common::MotionDirection::Forward ? 1 : 2)
            : 0,
        static_cast<int>(node.last_turn_class),
        discretizeMetric(node.same_motion_length_m),
        discretizeMetric(node.same_motion_remaining_to_change_m),
        node.active_transition.isActive()
            ? static_cast<int>(node.active_transition.source_frontier_id) + 1
            : 0,
        node.active_transition.isActive()
            ? static_cast<int>(std::lround(
                  node.active_transition.entry_station_m / xy_resolution_m))
            : 0
    };
}

double computeGlobalMaxSameMotionLength(const PlannerBehaviorSet& behavior_set) {
    double max_length = 0.0;
    for (const auto& behavior_name : behavior_set.names()) {
        max_length = std::max(
            max_length,
            behavior_set.get(behavior_name).planner.min_path_len_in_same_motion);
    }
    return max_length;
}

robot::RobotState makeRobotState(const math::Pose2d& pose) {
    robot::RobotState state;
    state.x = pose.x;
    state.y = pose.y;
    state.yaw = pose.theta.radians();
    return state;
}

double elapsedMilliseconds(const Clock::time_point& start,
                           const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string zoneLabel(const std::shared_ptr<zones::Zone>& zone) {
    if (zone == nullptr) {
        return "";
    }
    if (zone->getName().has_value() && !zone->getName()->empty()) {
        return zone->getName().value();
    }
    if (std::dynamic_pointer_cast<zones::TrackMainRoad>(zone) != nullptr) {
        return "TrackMainRoad";
    }
    if (std::dynamic_pointer_cast<zones::ManeuveringZone>(zone) != nullptr) {
        return "ManeuveringZone";
    }
    return "Zone";
}

std::string frontierRoleName(costs::SearchFrontierRole role) {
    switch (role) {
    case costs::SearchFrontierRole::StartZone:
        return "start_zone";
    case costs::SearchFrontierRole::GoalZone:
        return "goal_zone";
    }
    return "frontier";
}

std::string turnDirectionName(TurnDirection turn_direction) {
    switch (turn_direction) {
    case TurnDirection::FORWARD:
        return "FORWARD";
    case TurnDirection::LEFT:
        return "LEFT";
    case TurnDirection::RIGHT:
        return "RIGHT";
    case TurnDirection::REVERSE:
        return "REVERSE";
    case TurnDirection::REV_LEFT:
        return "REV_LEFT";
    case TurnDirection::REV_RIGHT:
        return "REV_RIGHT";
    case TurnDirection::UNKNOWN:
        return "UNKNOWN";
    }
    return "UNKNOWN";
}

std::string transitionPromotionReasonName(
    TransitionPromotionReasonCode promotion_reason) {
    switch (promotion_reason) {
    case TransitionPromotionReasonCode::AlignedAndDeepEnough:
        return "aligned_and_deep_enough";
    case TransitionPromotionReasonCode::MaxDepthForced:
        return "max_depth_forced";
    case TransitionPromotionReasonCode::None:
        break;
    }
    return "";
}

std::string frontierGuidanceObjectiveName(FrontierGuidanceObjectiveKind objective_kind) {
    switch (objective_kind) {
    case FrontierGuidanceObjectiveKind::Goal:
        return "goal";
    case FrontierGuidanceObjectiveKind::Handoff:
        return "handoff";
    }
    return "goal";
}

std::string frontierHeuristicModeName(FrontierHeuristicMode mode) {
    switch (mode) {
    case FrontierHeuristicMode::GoalGrid:
        return "goal_grid";
    case FrontierHeuristicMode::InterfaceGrid:
        return "interface_grid";
    case FrontierHeuristicMode::DirectPoseFallback:
        return "direct_pose_fallback";
    }
    return "goal_grid";
}

double readLayerCost(const grid_map::GridMap& costmap,
                     const std::string& layer,
                     const grid_map::Position& position) {
    if (!costmap.exists(layer) || !costmap.isInside(position)) {
        return 0.0;
    }

    const float value = costmap.atPosition(
        layer, position, grid_map::InterpolationMethods::INTER_NEAREST);
    if (std::isnan(value)) {
        return 0.0;
    }
    return static_cast<double>(value);
}

double readRawLayerValue(const grid_map::GridMap& costmap,
                         const std::string& layer,
                         const grid_map::Position& position) {
    if (!costmap.exists(layer) || !costmap.isInside(position)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return static_cast<double>(costmap.atPosition(
        layer, position, grid_map::InterpolationMethods::INTER_NEAREST));
}

double computeHolonomicHeuristic(const costs::PlanningGridBundle& bundle,
                                 const grid_map::Position& position,
                                 const FrontierGuidanceContextRuntime* guidance,
                                 common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.holonomic_heuristic_read");
    if (guidance != nullptr && guidance->valid) {
        switch (guidance->heuristic_mode) {
        case FrontierHeuristicMode::DirectPoseFallback:
            return std::hypot(
                guidance->representative_pose.x - position.x(),
                guidance->representative_pose.y - position.y());
        case FrontierHeuristicMode::InterfaceGrid: {
            if (!guidance->has_interface_heuristic_field || guidance->interface_heuristic_field.size() == 0) {
                break;
            }
            grid_map::Index index;
            if (!bundle.heuristic_grid.getIndex(position, index)) {
                return LARGE_COST;
            }
            const float value = guidance->interface_heuristic_field(index(0), index(1));
            if (std::isnan(value)) {
                return LARGE_COST;
            }
            return static_cast<double>(value);
        }
        case FrontierHeuristicMode::GoalGrid:
            break;
        }
    }

    if (!bundle.heuristic_grid.exists(costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES) ||
        !bundle.heuristic_grid.isInside(position)) {
        return LARGE_COST;
    }

    const float value = bundle.heuristic_grid.atPosition(
        costs::CostmapLayerNames::HOLONOMIC_WITH_OBSTACLES,
        position,
        grid_map::InterpolationMethods::INTER_NEAREST);
    if (std::isnan(value)) {
        return LARGE_COST;
    }
    return static_cast<double>(value);
}

double computePlannerHeuristic(
    const costs::PlanningGridBundle& bundle,
    const DualModelNonHolonomicHeuristic& heuristic,
    HeuristicModel model,
    const math::Pose2d& pose,
    const math::Pose2d& fallback_target_pose,
    const FrontierGuidanceContextRuntime* guidance,
    common::ProfilingCollector* profiler = nullptr) {
    const double holonomic_h = computeHolonomicHeuristic(
        bundle,
        grid_map::Position(pose.x, pose.y),
        guidance,
        profiler);

    const FrontierHeuristicMode mode =
        (guidance != nullptr && guidance->valid) ? guidance->heuristic_mode
                                                 : FrontierHeuristicMode::GoalGrid;
    if (mode == FrontierHeuristicMode::InterfaceGrid) {
        return holonomic_h;
    }

    const math::Pose2d& nh_target =
        (guidance != nullptr && guidance->valid)
            ? guidance->representative_pose
            : fallback_target_pose;
    return std::max(
        holonomic_h,
        computeNonHolonomicHeuristic(
            heuristic,
            model,
            pose,
            nh_target,
            profiler));
}

double computeFrontierGuidanceHint(
    const costs::PlanningGridBundle& bundle,
    const grid_map::Position& position,
    const FrontierGuidanceContextRuntime* guidance) {
    if (guidance == nullptr || !guidance->valid) {
        return 0.0;
    }

    if (guidance->heuristic_mode != FrontierHeuristicMode::GoalGrid ||
        !guidance->has_interface_heuristic_field ||
        guidance->interface_heuristic_field.size() == 0) {
        return 0.0;
    }

    {
        grid_map::Index index;
        if (bundle.heuristic_grid.getIndex(position, index)) {
            const float value = guidance->interface_heuristic_field(index(0), index(1));
            if (std::isfinite(value)) {
                return static_cast<double>(value);
            }
        }
    }
    return 0.0;
}

double computeNonHolonomicHeuristic(const DualModelNonHolonomicHeuristic& heuristic,
                                    HeuristicModel model,
                                    const math::Pose2d& from,
                                    const math::Pose2d& goal,
                                    common::ProfilingCollector* profiler) {
    common::ScopedProfilingTimer timer(profiler, "planner.non_holonomic_heuristic_lookup");
    const double dx_world = goal.x - from.x;
    const double dy_world = goal.y - from.y;
    const double cos_yaw = std::cos(from.theta.radians());
    const double sin_yaw = std::sin(from.theta.radians());

    const double rel_x = cos_yaw * dx_world + sin_yaw * dy_world;
    const double rel_y = -sin_yaw * dx_world + cos_yaw * dy_world;
    const double rel_theta = normalizeAngleSigned(goal.theta.radians() - from.theta.radians());

    return heuristic.lookup(
        model,
        static_cast<float>(rel_x),
        static_cast<float>(rel_y),
        static_cast<float>(rel_theta));
}

double computeEdgeCost(const costs::PlanningGridBundle& bundle,
                       const PlannerBehaviorProfile& profile,
                       const SearchNode& parent,
                       const grid_map::Position& successor_position,
                       double lane_guidance_scale,
                       double successor_yaw_rad,
                       TurnDirection turn_direction,
                       double travel_m) {
    double edge_cost = travel_m;
    const common::MotionDirection successor_motion =
        motionDirectionForPrimitive(turn_direction);
    const bool reverse_motion = successor_motion == common::MotionDirection::Reverse;

    edge_cost *= reverse_motion
        ? profile.planner.weight_reverse
        : profile.planner.weight_forward;

    const TurnClass turn_class = classifyTurn(turn_direction);
    if (turn_class == TurnClass::LEFT || turn_class == TurnClass::RIGHT) {
        edge_cost += profile.planner.weight_steer * travel_m;
    }

    if (parent.last_turn_class != TurnClass::UNKNOWN &&
        parent.last_turn_class != turn_class) {
        edge_cost += profile.planner.weight_steer_change * travel_m;
    }

    if (parent.has_inbound_motion && parent.inbound_motion != successor_motion) {
        edge_cost += profile.planner.weight_gear_change;
    }

    if (profile.isLayerActive(costs::CostmapLayerNames::INFLATION)) {
        edge_cost +=
            travel_m *
            (readLayerCost(
                 bundle.guidance_grid,
                 costs::CostmapLayerNames::INFLATION,
                 successor_position) /
             CostValues::LETHAL);
        edge_cost +=
            travel_m *
            (readLayerCost(
                 bundle.dynamic_grid,
                 costs::CostmapLayerNames::DYNAMIC_INFLATION,
                 successor_position) /
             CostValues::LETHAL);
    }

    if (profile.isLayerActive(costs::CostmapLayerNames::LANE_CENTERLINE_COST)) {
        edge_cost += travel_m * profile.planner.weight_lane_centerline * lane_guidance_scale *
            (readLayerCost(bundle.guidance_grid,
                           costs::CostmapLayerNames::LANE_CENTERLINE_COST,
                           successor_position) / CostValues::LETHAL);
    }

    if (profile.planner.lane_heading_bias_weight > 0.0) {
        const double lane_heading = readRawLayerValue(
            bundle.guidance_grid,
            costs::CostmapLayerNames::LANE_HEADING,
            successor_position);
        if (std::isfinite(lane_heading)) {
            const double heading_error =
                std::abs(normalizeAngleSigned(successor_yaw_rad - lane_heading));
            edge_cost +=
                travel_m * profile.planner.lane_heading_bias_weight *
                lane_guidance_scale * heading_error;
        }
    }

    return edge_cost;
}

TrackLaneRelaxationState computeTrackLaneRelaxation(
    const costs::PlanningGridBundle& bundle,
    const std::vector<FrontierTrackLaneGuidanceRuntime>& frontier_guidance,
    size_t frontier_id,
    const grid_map::Position& position) {
    TrackLaneRelaxationState state;
    if (frontier_id >= frontier_guidance.size()) {
        return state;
    }

    const auto& guidance = frontier_guidance[frontier_id];
    if (!guidance.enabled || !bundle.guidance_grid.isInside(position) ||
        !bundle.guidance_grid.exists(costs::CostmapLayerNames::TRACK_STATION) ||
        !bundle.guidance_grid.exists(costs::CostmapLayerNames::ZONE_CONSTRAINTS)) {
        return state;
    }

    const double zone_owner = readRawLayerValue(
        bundle.guidance_grid, costs::CostmapLayerNames::ZONE_CONSTRAINTS, position);
    if (!std::isfinite(zone_owner) ||
        std::abs(zone_owner - static_cast<double>(frontier_id)) >= 0.5) {
        return state;
    }

    const double station = readRawLayerValue(
        bundle.guidance_grid, costs::CostmapLayerNames::TRACK_STATION, position);
    if (!std::isfinite(station)) {
        return state;
    }

    state.active = true;
    state.station_m = station;
    state.distance_to_target_m = std::abs(guidance.target_station_m - station);
    if (guidance.relax_window_m <= common::EPSILON) {
        return state;
    }

    if (state.distance_to_target_m > guidance.relax_window_m) {
        return state;
    }

    state.inside_relax_window = true;
    state.bias_scale = std::clamp(
        state.distance_to_target_m / guidance.relax_window_m,
        0.0,
        1.0);
    return state;
}

bool canChangeMotionDirection(const SearchNode& node,
                              common::MotionDirection successor_motion) {
    return !node.has_inbound_motion ||
           node.inbound_motion == successor_motion ||
           node.same_motion_remaining_to_change_m <= common::EPSILON;
}

double computeNextSameMotionLength(const SearchNode& parent,
                                   common::MotionDirection successor_motion,
                                   double travel_m,
                                   double global_max_same_motion_length_m) {
    const double unclamped_length =
        (!parent.has_inbound_motion || parent.inbound_motion != successor_motion)
        ? travel_m
        : parent.same_motion_length_m + travel_m;
    if (global_max_same_motion_length_m <= common::EPSILON) {
        return 0.0;
    }
    return std::min(unclamped_length, global_max_same_motion_length_m);
}

double computeNextSameMotionRemainingToChange(
    const SearchNode& parent,
    const PlannerBehaviorProfile& current_profile,
    const PlannerBehaviorProfile& successor_profile,
    common::MotionDirection successor_motion,
    double travel_m) {
    if (!parent.has_inbound_motion || parent.inbound_motion != successor_motion) {
        const double new_segment_min = std::max(
            current_profile.planner.min_path_len_in_same_motion,
            successor_profile.planner.min_path_len_in_same_motion);
        return std::max(0.0, new_segment_min - travel_m);
    }

    return std::max(
        0.0,
        std::max(
            parent.same_motion_remaining_to_change_m,
            successor_profile.planner.min_path_len_in_same_motion -
                parent.same_motion_length_m) -
            travel_m);
}

bool isTerminalMotionSegmentValid(const SearchNode& node) {
    return !node.has_inbound_motion ||
           node.same_motion_remaining_to_change_m <= common::EPSILON;
}

bool allowsReverseMotion(const PlannerBehaviorProfile& profile,
                         const std::shared_ptr<zones::Zone>& zone) {
    return !profile.planner.only_forward_path &&
           (zone == nullptr || zone->isReverseAllowed());
}

bool allowsReverseMotion(const ResolvedPlannerBehavior& resolved_behavior) {
    return resolved_behavior.profile != nullptr &&
           allowsReverseMotion(*resolved_behavior.profile, resolved_behavior.zone);
}

AnalyticPathValidationResult validateAnalyticPathSegments(
    const SearchNode& start_node,
    const std::vector<std::array<double, 3>>& waypoints,
    const std::vector<ResolvedPlannerBehavior>& waypoint_resolutions,
    double global_max_same_motion_length_m) {
    AnalyticPathValidationResult result;
    if (waypoints.size() < 2 || waypoints.size() != waypoint_resolutions.size()) {
        return result;
    }

    SearchNode simulated_node = start_node;
    common::MotionDirection fallback_direction = simulated_node.has_inbound_motion
        ? simulated_node.inbound_motion
        : common::MotionDirection::Forward;
    result.segment_directions.reserve(waypoints.size() - 1);

    for (size_t i = 1; i < waypoints.size(); ++i) {
        const auto& from = waypoints[i - 1];
        const auto& to = waypoints[i];
        const ResolvedPlannerBehavior& current_resolution = waypoint_resolutions[i - 1];
        const ResolvedPlannerBehavior& successor_resolution = waypoint_resolutions[i];
        if (current_resolution.profile == nullptr || successor_resolution.profile == nullptr) {
            return AnalyticPathValidationResult{};
        }

        const double travel_m = std::hypot(to[0] - from[0], to[1] - from[1]);
        const math::Pose2d from_pose(
            from[0], from[1], math::Angle::from_radians(from[2]));
        const math::Pose2d to_pose(
            to[0], to[1], math::Angle::from_radians(to[2]));
        const common::MotionDirection step_motion =
            from_pose.inferMotionDirectionTo(to_pose, fallback_direction);

        if (step_motion == common::MotionDirection::Reverse &&
            !allowsReverseMotion(*successor_resolution.profile, successor_resolution.zone)) {
            return AnalyticPathValidationResult{};
        }

        if (!canChangeMotionDirection(simulated_node, step_motion)) {
            return AnalyticPathValidationResult{};
        }

        simulated_node.same_motion_length_m = computeNextSameMotionLength(
            simulated_node,
            step_motion,
            travel_m,
            global_max_same_motion_length_m);
        simulated_node.same_motion_remaining_to_change_m =
            computeNextSameMotionRemainingToChange(
                simulated_node,
                *current_resolution.profile,
                *successor_resolution.profile,
                step_motion,
                travel_m);
        simulated_node.has_inbound_motion = true;
        simulated_node.inbound_motion = step_motion;
        simulated_node.zone = successor_resolution.zone;
        simulated_node.behavior_name = successor_resolution.behavior_name;
        simulated_node.pose = to_pose;

        fallback_direction = step_motion;
        result.segment_directions.push_back(step_motion);
    }

    result.valid = isTerminalMotionSegmentValid(simulated_node);
    if (!result.valid) {
        result.segment_directions.clear();
    }
    return result;
}

bool isPoseSegmentStraightEnough(const math::Pose2d& from,
                                 const math::Pose2d& to,
                                 common::MotionDirection direction) {
    const double yaw_delta =
        std::abs(normalizeAngleSigned(to.theta.radians() - from.theta.radians()));
    if (yaw_delta > kTerminalStraightYawDeltaToleranceRad) {
        return false;
    }

    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    if (std::hypot(dx, dy) <= common::EPSILON) {
        return false;
    }

    double expected_heading = from.theta.radians();
    if (direction == common::MotionDirection::Reverse) {
        expected_heading = normalizeAngleSigned(expected_heading + common::PI);
    }

    const double segment_heading = std::atan2(dy, dx);
    const double alignment_error =
        std::abs(normalizeAngleSigned(segment_heading - expected_heading));
    return alignment_error <= kTerminalStraightAlignmentToleranceRad;
}

double computeTerminalStraightApproachLength(
    const std::vector<math::Pose2d>& poses,
    const std::vector<common::MotionDirection>& segment_directions) {
    if (poses.size() < 2 || segment_directions.size() + 1 != poses.size()) {
        return 0.0;
    }

    const common::MotionDirection terminal_direction = segment_directions.back();
    double straight_length_m = 0.0;
    for (size_t i = segment_directions.size(); i-- > 0;) {
        if (segment_directions[i] != terminal_direction) {
            break;
        }
        if (!isPoseSegmentStraightEnough(poses[i], poses[i + 1], segment_directions[i])) {
            break;
        }
        straight_length_m += std::hypot(
            poses[i + 1].x - poses[i].x,
            poses[i + 1].y - poses[i].y);
    }
    return straight_length_m;
}

double goalCandidateProgress(const GoalCandidate& candidate) {
    if (!candidate.valid) {
        return -1.0;
    }
    if (candidate.required_straight_approach_m <= common::EPSILON) {
        return 1.0;
    }
    return candidate.terminal_straight_approach_m /
        candidate.required_straight_approach_m;
}

bool shouldReplaceGoalCandidate(const GoalCandidate& current_best,
                                const GoalCandidate& candidate) {
    if (!candidate.valid) {
        return false;
    }
    if (!current_best.valid) {
        return true;
    }

    const double current_progress = goalCandidateProgress(current_best);
    const double candidate_progress = goalCandidateProgress(candidate);
    if (candidate_progress > current_progress + 1e-6) {
        return true;
    }
    if (candidate_progress + 1e-6 < current_progress) {
        return false;
    }

    if (candidate.terminal_straight_approach_m >
        current_best.terminal_straight_approach_m + 1e-6) {
        return true;
    }
    if (candidate.terminal_straight_approach_m + 1e-6 <
        current_best.terminal_straight_approach_m) {
        return false;
    }

    return candidate.path_cost + 1e-6 < current_best.path_cost;
}

std::string buildGoalCandidateFallbackDetail(const GoalCandidate& candidate,
                                             const std::string& reason) {
    std::ostringstream detail;
    detail << candidate.result.detail << " Returned best goal candidate after " << reason
           << " without meeting planner.min_goal_straight_approach_m="
           << candidate.required_straight_approach_m << " m"
           << " (best terminal straight approach="
           << candidate.terminal_straight_approach_m << " m).";
    return detail.str();
}

bool isGoalSatisfied(const SearchNode& node,
                     const math::Pose2d& goal,
                     double goal_distance_tolerance,
                     double yaw_bin_size_rad) {
    const double dx = goal.x - node.pose.x;
    const double dy = goal.y - node.pose.y;
    const double distance = std::hypot(dx, dy);
    const double heading_error =
        std::abs(normalizeAngleSigned(goal.theta.radians() - node.pose.theta.radians()));

    return distance <= goal_distance_tolerance &&
           heading_error <= yaw_bin_size_rad;
}

HybridAStarPlannerResult reconstructResult(const std::deque<SearchNode>& nodes,
                                           int goal_index,
                                           common::ProfilingCollector* profiler = nullptr) {
    common::ScopedProfilingTimer timer(profiler, "planner.result_reconstruction");
    HybridAStarPlannerResult result;
    result.success = true;

    std::vector<math::Pose2d> reversed_poses;
    std::vector<std::string> reversed_behaviors;
    std::vector<common::MotionDirection> reversed_directions;

    int current_index = goal_index;
    while (current_index >= 0) {
        const SearchNode& node = nodes[static_cast<size_t>(current_index)];
        reversed_poses.push_back(node.pose);
        reversed_behaviors.push_back(node.behavior_name);
        if (node.parent_index >= 0) {
            reversed_directions.push_back(node.inbound_motion);
        }
        current_index = node.parent_index;
    }

    result.poses.assign(reversed_poses.rbegin(), reversed_poses.rend());
    result.behavior_sequence.assign(reversed_behaviors.rbegin(), reversed_behaviors.rend());
    result.segment_directions.assign(reversed_directions.rbegin(), reversed_directions.rend());
    result.detail = "Path found.";
    return result;
}

SearchContext buildSearchContext(const std::string& behavior_name,
                                 const PlannerBehaviorSet& behavior_set) {
    SearchContext context;
    context.behavior_name = behavior_name;
    context.profile = &behavior_set.get(behavior_name);
    context.xy_resolution_m = context.profile->planner.xy_grid_resolution_m;

    MotionTableConfig motion_config;
    motion_config.minimum_turning_radius = static_cast<float>(
        context.profile->motion_primitives.min_turning_radius_m /
        context.profile->planner.xy_grid_resolution_m);
    motion_config.num_angle_quantization = static_cast<unsigned int>(
        context.profile->motion_primitives.num_angle_bins);
    context.motion_table.initReedsShepp(motion_config);
    context.yaw_bin_size_rad = context.motion_table.getBinSize();
    context.goal_distance_tolerance = context.profile->planner.step_size_m;
    context.analytic_max_length_m =
        context.profile->planner.analytic_expansion_max_length_m;
    const double analytic_ratio = context.profile->planner.analytic_expansion_ratio;
    context.analytic_expansion_interval = std::max(
        1u, static_cast<unsigned int>(
                std::round(1.0 / std::max(analytic_ratio, 0.01))));
    context.analytic_step_size = context.xy_resolution_m * 2.0;
    return context;
}

void mergeFrontierCountersIntoTrace(const FrontierCounters& counters,
                                    HybridAStarPlannerDebugTrace* trace) {
    if (trace == nullptr) {
        return;
    }

    trace->expansions_popped += counters.expansions_popped;
    trace->stale_entries_skipped += counters.stale_entries_skipped;
    trace->goal_checks += counters.goal_checks;
    trace->goal_hits += counters.goal_hits;
    trace->lane_following_candidates += counters.lane_following_candidates;
    trace->lane_suppression_forward_only_applied +=
        counters.lane_suppression_forward_only_applied;
    trace->lane_suppression_fallbacks += counters.lane_suppression_fallbacks;
    trace->analytic_attempts += counters.analytic_attempts;
    trace->analytic_successes += counters.analytic_successes;
    trace->analytic_fail_no_ompl += counters.analytic_fail_no_ompl;
    trace->analytic_fail_same_motion_guard += counters.analytic_fail_same_motion_guard;
    trace->analytic_fail_path_length += counters.analytic_fail_path_length;
    trace->analytic_fail_out_of_bounds += counters.analytic_fail_out_of_bounds;
    trace->analytic_fail_collision += counters.analytic_fail_collision;
    trace->analytic_fail_validation += counters.analytic_fail_validation;
    trace->primitive_out_of_bounds += counters.primitive_out_of_bounds;
    trace->primitive_cross_track_pruned += counters.primitive_cross_track_pruned;
    trace->primitive_behavior_unresolved += counters.primitive_behavior_unresolved;
    trace->primitive_disallowed += counters.primitive_disallowed;
    trace->primitive_collision += counters.primitive_collision;
    trace->primitive_motion_change_blocked += counters.primitive_motion_change_blocked;
    trace->primitive_dominated += counters.primitive_dominated;
    trace->primitive_enqueued += counters.primitive_enqueued;
}

} // namespace

HybridAStarPlanner::HybridAStarPlanner(
    const robot::Car& car,
    std::vector<std::shared_ptr<zones::Zone>> all_zones,
    PlannerBehaviorSet behavior_set,
    std::shared_ptr<const costs::MapLayerCache> map_layer_cache)
    : car_(car),
      all_zones_(std::move(all_zones)),
      behavior_set_(std::move(behavior_set)),
      map_layer_cache_(std::move(map_layer_cache)) {}

HybridAStarPlannerResult HybridAStarPlanner::plan(
    const HybridAStarPlannerRequest& request) const {
    const Clock::time_point plan_start_time = Clock::now();
    HybridAStarPlannerResult failure_result;
    const std::shared_ptr<HybridAStarPlannerDebugTrace> debug_trace =
        behavior_set_.debugModeEnabled()
            ? std::make_shared<HybridAStarPlannerDebugTrace>()
            : nullptr;
    common::ProfilingCollector attempt_profiling_collector;
    common::ProfilingCollector* profiler =
        debug_trace != nullptr ? &attempt_profiling_collector : nullptr;
    Clock::time_point search_loop_start_time{};
    bool search_loop_started = false;
    std::vector<std::unique_ptr<FrontierRuntime>> frontier_runtimes;
    std::unordered_map<uint64_t, PlannerFrontierHandoffDebugSummary> handoff_summaries;

    if (debug_trace != nullptr) {
        debug_trace->initial_behavior_name = request.initial_behavior_name;
    }

    const auto finalizeResult =
        [&](HybridAStarPlannerResult result,
            const std::deque<SearchNode>& nodes,
            size_t queue_peak_size) -> HybridAStarPlannerResult {
        if (debug_trace != nullptr) {
            if (search_loop_started) {
                debug_trace->search_loop_ms =
                    elapsedMilliseconds(search_loop_start_time, Clock::now());
            }
            debug_trace->nodes_allocated = nodes.size();
            debug_trace->unique_state_count = 0;
            debug_trace->open_queue_peak_size = queue_peak_size;
            for (const auto& runtime_ptr : frontier_runtimes) {
                auto& runtime = *runtime_ptr;
                {
                    std::lock_guard<std::mutex> lock(runtime.mutex);
                    runtime.debug_summary.closed_set_size = runtime.best_g_by_key.size();
                    runtime.debug_summary.profiling_scopes =
                        runtime.profiling_collector.snapshotSortedByTotalMs();
                    debug_trace->unique_state_count += runtime.best_g_by_key.size();
                }
                mergeFrontierCountersIntoTrace(runtime.counters, debug_trace.get());
                debug_trace->frontier_summaries.push_back(runtime.debug_summary);
                debug_trace->expansions.insert(
                    debug_trace->expansions.end(),
                    runtime.expansion_events.begin(),
                    runtime.expansion_events.end());
                attempt_profiling_collector.mergeFrom(runtime.profiling_collector);
            }
            std::sort(
                debug_trace->expansions.begin(),
                debug_trace->expansions.end(),
                [](const PlannerExpansionDebugEvent& lhs,
                   const PlannerExpansionDebugEvent& rhs) {
                    return lhs.expansion_index < rhs.expansion_index;
                });
            for (const auto& entry : handoff_summaries) {
                debug_trace->frontier_handoffs.push_back(entry.second);
            }
            std::sort(
                debug_trace->frontier_handoffs.begin(),
                debug_trace->frontier_handoffs.end(),
                [](const PlannerFrontierHandoffDebugSummary& lhs,
                   const PlannerFrontierHandoffDebugSummary& rhs) {
                    if (lhs.from_frontier_id != rhs.from_frontier_id) {
                        return lhs.from_frontier_id < rhs.from_frontier_id;
                    }
                    return lhs.to_frontier_id < rhs.to_frontier_id;
                });
            debug_trace->total_planning_ms =
                elapsedMilliseconds(plan_start_time, Clock::now());
            debug_trace->profiling_scopes =
                attempt_profiling_collector.snapshotSortedByTotalMs();
            result.debug_trace = debug_trace;
        }
        return result;
    };

    const auto failWithDetail =
        [&](std::string detail,
            const std::deque<SearchNode>& nodes = {}) -> HybridAStarPlannerResult {
        failure_result.detail = std::move(detail);
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason = failure_result.detail;
        }
        return finalizeResult(failure_result, nodes, 0);
    };

    if (!behavior_set_.contains(request.initial_behavior_name)) {
        return failWithDetail(
            "Initial planner behavior '" + request.initial_behavior_name + "' is not defined.");
    }
    const PlannerBehaviorProfile& initial_profile =
        behavior_set_.get(request.initial_behavior_name);
    if (initial_profile.motion_primitives.min_turning_radius_m <= 0.0 ||
        initial_profile.motion_primitives.max_steer_angle_rad <= 0.0) {
        return failWithDetail(
            "Motion primitive constraints were not initialized from the selected robot model.");
    }

    ZoneSelectionResult selection;
    try {
        const Clock::time_point selection_start_time = Clock::now();
        common::ScopedProfilingTimer selection_timer(profiler, "planner.zone_selection");
        costs::ZoneSelector selector;
        selection = selector.select(
            request.start,
            request.goal,
            all_zones_,
            request.initial_behavior_name);
        if (debug_trace != nullptr) {
            debug_trace->zone_selection_ms =
                elapsedMilliseconds(selection_start_time, Clock::now());
            for (const auto& zone : selection.selected_zones) {
                debug_trace->selected_zone_names.push_back(zoneLabel(zone));
            }
        }
    } catch (const std::exception& e) {
        return failWithDetail(e.what());
    }

    const auto start_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(request.start.x, request.start.y), selection.selected_zones);
    const auto goal_zone = costs::ZoneSelector::findContainingZone(
        geometry::Point2d(request.goal.x, request.goal.y), selection.selected_zones);
    if (debug_trace != nullptr) {
        debug_trace->start_zone_name = zoneLabel(start_zone);
        debug_trace->goal_zone_name = zoneLabel(goal_zone);
    }
    if (start_zone == nullptr) {
        return failWithDetail("Start pose is not inside the selected planning zones.");
    }

    costs::CostmapBuilder builder(
        initial_profile.makeCostmapConfig(),
        all_zones_,
        car_,
        behavior_set_.globalConfig().costmap_resolution_policy,
        map_layer_cache_,
        profiler);
    const Clock::time_point costmap_start_time = Clock::now();
    costs::PlanningGridBundle costmap_bundle =
        builder.buildBundle(selection, request.start, request.goal, request.obstacle_polygons);
    const grid_map::GridMap& costmap = costmap_bundle.guidance_grid;
    if (debug_trace != nullptr) {
        debug_trace->costmap_build_ms =
            elapsedMilliseconds(costmap_start_time, Clock::now());
    }

    CollisionCheckerConfig default_collision_config;
    default_collision_config.obstacle_layer = costs::CostmapLayerNames::STATIC_OBSTACLES;
    default_collision_config.lethal_threshold =
        static_cast<float>(initial_profile.collision_checker.lethal_threshold);
    CollisionChecker default_collision_checker(default_collision_config);
    CollisionCheckerConfig dynamic_collision_config;
    dynamic_collision_config.obstacle_layer = costs::CostmapLayerNames::DYNAMIC_OBSTACLES;
    dynamic_collision_config.lethal_threshold =
        static_cast<float>(initial_profile.collision_checker.lethal_threshold);
    CollisionChecker dynamic_collision_checker(dynamic_collision_config);
    const bool has_dynamic_obstacles = !request.obstacle_polygons.empty();

    const auto checkCompositeCollision =
        [&](const math::Pose2d& pose, float lethal_threshold, const std::string& profiling_scope) {
            const robot::RobotState state = makeRobotState(pose);
            const bool static_collision = [&]() {
                common::ScopedProfilingTimer timer(profiler, profiling_scope + ".static");
                if (std::abs(lethal_threshold - default_collision_config.lethal_threshold) <= 0.5f) {
                    return default_collision_checker.checkCollision(
                        costmap_bundle.guidance_grid, car_, state).in_collision;
                }

                CollisionCheckerConfig static_config = default_collision_config;
                static_config.lethal_threshold = lethal_threshold;
                CollisionChecker static_checker(static_config);
                return static_checker.checkCollision(
                    costmap_bundle.guidance_grid, car_, state).in_collision;
            }();
            if (static_collision) {
                return true;
            }
            if (!has_dynamic_obstacles) {
                return false;
            }

            const bool dynamic_collision = [&]() {
                common::ScopedProfilingTimer timer(profiler, profiling_scope + ".dynamic");
                if (std::abs(lethal_threshold - dynamic_collision_config.lethal_threshold) <= 0.5f) {
                    return dynamic_collision_checker.checkCollision(
                        costmap_bundle.dynamic_grid, car_, state).in_collision;
                }

                CollisionCheckerConfig dynamic_config = dynamic_collision_config;
                dynamic_config.lethal_threshold = lethal_threshold;
                CollisionChecker dynamic_checker(dynamic_config);
                return dynamic_checker.checkCollision(
                    costmap_bundle.dynamic_grid, car_, state).in_collision;
            }();
            return dynamic_collision;
        };

    const bool start_in_collision = [&]() {
        return checkCompositeCollision(
            request.start,
            default_collision_config.lethal_threshold,
            "planner.collision_check.start_validation");
    }();
    if (start_in_collision) {
        return failWithDetail("Start pose is in collision on the selected-zone costmap.");
    }
    const bool goal_in_collision = [&]() {
        return checkCompositeCollision(
            request.goal,
            default_collision_config.lethal_threshold,
            "planner.collision_check.goal_validation");
    }();
    if (goal_in_collision) {
        return failWithDetail("Goal pose is in collision on the selected-zone costmap.");
    }

    const Clock::time_point heuristic_setup_start_time = Clock::now();
    DualModelNonHolonomicHeuristic heuristic;
    heuristic.configureOmpl(
        static_cast<float>(initial_profile.motion_primitives.min_turning_radius_m));
    if (!request.dual_model_lut_path.empty()) {
        heuristic.loadFromFile(request.dual_model_lut_path);
    }

    std::unordered_map<std::string, SearchContext> contexts_by_behavior;
    for (const auto& behavior_name : behavior_set_.names()) {
        contexts_by_behavior.emplace(
            behavior_name,
            buildSearchContext(behavior_name, behavior_set_));
    }
    const auto getSearchContext =
        [&](const std::string& behavior_name) -> const SearchContext& {
            return contexts_by_behavior.at(behavior_name);
        };

    frontier_runtimes.reserve(selection.frontiers.size());
    for (const auto& frontier : selection.frontiers) {
        auto runtime = std::make_unique<FrontierRuntime>();
        runtime->descriptor = frontier;
        if (runtime->descriptor.behavior_name.empty()) {
            runtime->descriptor.behavior_name = request.initial_behavior_name;
        }
        runtime->context = &getSearchContext(runtime->descriptor.behavior_name);
        runtime->best_g_by_key.reserve(50000);
        runtime->debug_summary.frontier_id = runtime->descriptor.frontier_id;
        runtime->debug_summary.frontier_role =
            frontierRoleName(runtime->descriptor.role);
        runtime->debug_summary.zone_name = zoneLabel(runtime->descriptor.zone);
        runtime->debug_summary.behavior_name = runtime->descriptor.behavior_name;
        frontier_runtimes.push_back(std::move(runtime));
    }

    std::vector<FrontierTrackLaneGuidanceRuntime> frontier_track_lane_guidance(
        selection.frontiers.size());
    for (const auto& guidance : builder.getTrackLaneGuidance()) {
        if (guidance.frontier_id >= frontier_runtimes.size()) {
            continue;
        }

        const SearchContext& guidance_context = *frontier_runtimes[guidance.frontier_id]->context;
        const PlannerBehaviorProfile& guidance_profile = *guidance_context.profile;
        FrontierTrackLaneGuidanceRuntime runtime_guidance;
        runtime_guidance.enabled = true;
        runtime_guidance.target_station_m = guidance.target_station;
        runtime_guidance.target_kind = guidance.target_kind;
        runtime_guidance.relax_window_m =
            guidance.target_kind == costs::TrackLaneGuidanceTargetKind::Goal
                ? std::max(
                      guidance_profile.planner.min_goal_straight_approach_m,
                      guidance_profile.planner.step_size_m)
                : std::max({
                      2.0 * guidance_profile.planner.step_size_m,
                      guidance_profile.planner.max_cross_track_error_m,
                      guidance_context.xy_resolution_m
                  });
        frontier_track_lane_guidance[guidance.frontier_id] = runtime_guidance;
    }

    std::vector<FrontierGuidanceContextRuntime> frontier_guidance_contexts(
        selection.frontiers.size());
    std::vector<PlannerFrontierHeuristicDebugSummary> frontier_heuristic_summaries;
    frontier_heuristic_summaries.reserve(selection.frontiers.size());
    for (size_t frontier_index = 0; frontier_index < selection.frontiers.size(); ++frontier_index) {
        const auto& frontier = selection.frontiers[frontier_index];
        const FrontierGuidanceObjectiveKind default_objective_kind =
            frontier_index + 1 < selection.frontiers.size()
                ? FrontierGuidanceObjectiveKind::Handoff
                : FrontierGuidanceObjectiveKind::Goal;
        PlannerFrontierHeuristicDebugSummary summary;
        summary.frontier_id = frontier.frontier_id;
        summary.zone_name = zoneLabel(frontier.zone);
        summary.objective_kind = frontierGuidanceObjectiveName(default_objective_kind);
        const FrontierHeuristicMode default_mode =
            frontier_index + 1 < selection.frontiers.size()
                ? FrontierHeuristicMode::GoalGrid
                : FrontierHeuristicMode::GoalGrid;
        summary.uses_holonomic_grid = default_mode != FrontierHeuristicMode::DirectPoseFallback;
        summary.target_mode = frontierHeuristicModeName(default_mode);
        summary.reason = "default";
        summary.target_pose = request.goal;
        summary.seed_count = 0;
        frontier_heuristic_summaries.push_back(std::move(summary));
        if (frontier.frontier_id < frontier_guidance_contexts.size()) {
            frontier_guidance_contexts[frontier.frontier_id].valid = true;
            frontier_guidance_contexts[frontier.frontier_id].objective_kind =
                default_objective_kind;
            frontier_guidance_contexts[frontier.frontier_id].heuristic_mode =
                default_mode;
            frontier_guidance_contexts[frontier.frontier_id].representative_pose = request.goal;
        }
    }
    for (size_t frontier_index = 0; frontier_index + 1 < selection.frontiers.size();
         ++frontier_index) {
        const auto& current_frontier = selection.frontiers[frontier_index];
        const auto& next_frontier = selection.frontiers[frontier_index + 1];
        if (current_frontier.zone == nullptr || next_frontier.zone == nullptr ||
            current_frontier.frontier_id >= frontier_guidance_contexts.size()) {
            continue;
        }

        FrontierGuidanceContextRuntime& guidance =
            frontier_guidance_contexts[current_frontier.frontier_id];
        guidance.objective_kind = FrontierGuidanceObjectiveKind::Handoff;

        const auto destination_track =
            std::dynamic_pointer_cast<zones::TrackMainRoad>(next_frontier.zone);
        if (destination_track == nullptr ||
            next_frontier.frontier_id >= frontier_track_lane_guidance.size() ||
            !frontier_track_lane_guidance[next_frontier.frontier_id].enabled) {
            continue;
        }

        const auto* transition_policy = behavior_set_.findTransitionPolicy(
            zones::zoneTypeName(current_frontier.zone),
            zones::zoneTypeName(next_frontier.zone));
        if (transition_policy == nullptr) {
            continue;
        }

        const auto handoff_pose = computeGoalDirectedTrackEntryPose(
            current_frontier.zone,
            destination_track,
            request.goal,
            frontier_track_lane_guidance[next_frontier.frontier_id].target_station_m,
            transition_policy->min_depth_m);
        if (!handoff_pose.has_value()) {
            continue;
        }

        if (current_frontier.frontier_id == selection.frontiers.front().frontier_id) {
            const bool direct_line_hits_handoff = directGoalSegmentHitsHandoffRegion(
                request.start,
                request.goal,
                current_frontier.zone->getPolygon(),
                destination_track->getPolygon());
            const auto interface_seeds = selectInterfaceSeedPositions(
                current_frontier,
                next_frontier,
                *handoff_pose);
            const double goal_bearing_rad = std::atan2(
                request.goal.y - request.start.y,
                request.goal.x - request.start.x);
            const double goal_heading_delta_rad = std::abs(normalizeAngleSigned(
                goal_bearing_rad - request.start.theta.radians()));
            const double entry_heading_delta_rad = std::abs(normalizeAngleSigned(
                handoff_pose->theta.radians() - request.start.theta.radians()));
            const bool should_use_interface_bias =
                !direct_line_hits_handoff &&
                entry_heading_delta_rad <= (45.0 * common::DEG_TO_RAD) &&
                entry_heading_delta_rad + (5.0 * common::DEG_TO_RAD) <
                    goal_heading_delta_rad;
            const bool should_use_direct_entry_pose =
                !direct_line_hits_handoff &&
                interface_seeds.empty() &&
                goal_heading_delta_rad >= (90.0 * common::DEG_TO_RAD);
            if (interface_seeds.empty() && !should_use_direct_entry_pose) {
                continue;
            }
            guidance.valid = true;
            guidance.representative_pose = *handoff_pose;
            guidance.interface_seed_positions = interface_seeds;
            if (!interface_seeds.empty()) {
                guidance.heuristic_mode = should_use_interface_bias
                    ? FrontierHeuristicMode::GoalGrid
                    : FrontierHeuristicMode::InterfaceGrid;
                guidance.interface_heuristic_field =
                    costs::HolonomicObstaclesHeuristic::computeField(
                        costmap_bundle.heuristic_grid,
                        interface_seeds);
                guidance.has_interface_heuristic_field =
                    guidance.interface_heuristic_field.size() > 0;
                if (!guidance.has_interface_heuristic_field) {
                    guidance.heuristic_mode = FrontierHeuristicMode::DirectPoseFallback;
                }
            } else {
                guidance.heuristic_mode = FrontierHeuristicMode::DirectPoseFallback;
                guidance.has_interface_heuristic_field = false;
            }

            if (current_frontier.frontier_id < frontier_heuristic_summaries.size()) {
                auto& summary = frontier_heuristic_summaries[current_frontier.frontier_id];
                summary.objective_kind =
                    frontierGuidanceObjectiveName(FrontierGuidanceObjectiveKind::Handoff);
                summary.uses_holonomic_grid =
                    guidance.heuristic_mode != FrontierHeuristicMode::DirectPoseFallback;
                summary.target_mode =
                    guidance.heuristic_mode == FrontierHeuristicMode::GoalGrid &&
                        guidance.has_interface_heuristic_field
                        ? "goal_grid_with_interface_bias"
                        : frontierHeuristicModeName(guidance.heuristic_mode);
                summary.reason = "track_frontier_handoff";
                summary.target_pose = *handoff_pose;
                summary.seed_count = interface_seeds.size();
            }
            continue;
        }

        const auto interface_seeds = selectInterfaceSeedPositions(
            current_frontier,
            next_frontier,
            *handoff_pose);
        guidance.valid = true;
        guidance.representative_pose = *handoff_pose;
        guidance.interface_seed_positions = interface_seeds;
        if (!interface_seeds.empty()) {
            guidance.heuristic_mode = FrontierHeuristicMode::InterfaceGrid;
            guidance.interface_heuristic_field = costs::HolonomicObstaclesHeuristic::computeField(
                costmap_bundle.heuristic_grid,
                interface_seeds);
            guidance.has_interface_heuristic_field =
                guidance.interface_heuristic_field.size() > 0;
            if (!guidance.has_interface_heuristic_field) {
                guidance.heuristic_mode = FrontierHeuristicMode::DirectPoseFallback;
            }
        } else {
            guidance.heuristic_mode = FrontierHeuristicMode::DirectPoseFallback;
            guidance.has_interface_heuristic_field = false;
        }

        if (current_frontier.frontier_id < frontier_heuristic_summaries.size()) {
            auto& summary = frontier_heuristic_summaries[current_frontier.frontier_id];
            summary.objective_kind =
                frontierGuidanceObjectiveName(FrontierGuidanceObjectiveKind::Handoff);
            summary.uses_holonomic_grid =
                guidance.heuristic_mode != FrontierHeuristicMode::DirectPoseFallback;
            summary.target_mode = frontierHeuristicModeName(guidance.heuristic_mode);
            summary.reason = "track_frontier_handoff";
            summary.target_pose = *handoff_pose;
            summary.seed_count = interface_seeds.size();
        }
    }
    if (debug_trace != nullptr) {
        debug_trace->frontier_heuristics = frontier_heuristic_summaries;
        debug_trace->heuristic_setup_ms =
            elapsedMilliseconds(heuristic_setup_start_time, Clock::now());
    }

    const double global_max_same_motion_length_m =
        computeGlobalMaxSameMotionLength(behavior_set_);
    const auto deadline = Clock::now() + std::chrono::milliseconds(
        initial_profile.planner.max_planning_time_ms);
    search_loop_start_time = Clock::now();
    search_loop_started = true;

    const SearchContext& start_context =
        getSearchContext(frontier_runtimes.front()->descriptor.behavior_name);
    const double normalized_start_yaw = normalizeAngleUnsigned(request.start.theta.radians());
    const unsigned int start_heading_bin =
        start_context.motion_table.getClosestAngularBin(normalized_start_yaw);
    const HeuristicModel start_model = allowsReverseMotion(*start_context.profile, start_zone)
        ? HeuristicModel::REEDS_SHEPP
        : HeuristicModel::DUBINS;
    const FrontierGuidanceContextRuntime* start_guidance_context =
        selection.frontiers.empty() ||
                selection.frontiers.front().frontier_id >= frontier_guidance_contexts.size()
            ? nullptr
            : &frontier_guidance_contexts[selection.frontiers.front().frontier_id];
    const double start_h = computePlannerHeuristic(
        costmap_bundle,
        heuristic,
        start_model,
        request.start,
        request.goal,
        start_guidance_context,
        profiler);

    std::deque<SearchNode> nodes;
    nodes.push_back(SearchNode{
        request.start,
        0,
        start_heading_bin,
        0.0,
        start_h,
        -1,
        start_zone,
        frontier_runtimes.front()->descriptor.behavior_name,
        TurnClass::UNKNOWN,
        false,
        common::MotionDirection::Forward,
        0.0,
        0.0,
        ActiveZoneTransitionState{},
        TransitionPromotionReasonCode::None
    });

    const DiscreteStateKey start_key = discretizeState(
        request.start,
        start_heading_bin,
        nodes.front(),
        start_context.xy_resolution_m,
        global_max_same_motion_length_m);
    {
        auto& start_frontier = *frontier_runtimes.front();
        std::lock_guard<std::mutex> lock(start_frontier.mutex);
        start_frontier.best_g_by_key.emplace(start_key, 0.0);
        const double start_guidance_hint = computeFrontierGuidanceHint(
            costmap_bundle,
            grid_map::Position(request.start.x, request.start.y),
            start_guidance_context);
        start_frontier.open_queue.push(OpenEntry{
            nodes.front().g + nodes.front().h +
                (kInterfaceGuidanceTieBreakWeight * start_guidance_hint),
            nodes.front().g + nodes.front().h,
            nodes.front().h,
            nodes.front().g,
            start_guidance_hint,
            0,
            0,
            start_key
        });
        start_frontier.debug_summary.enqueued = 1;
        start_frontier.debug_summary.open_queue_peak_size = 1;
        start_frontier.debug_summary.first_enqueue_ms = 0.0;
    }

    std::mutex nodes_mutex;
    std::mutex result_mutex;
    std::mutex handoff_mutex;
    std::mutex scheduler_mutex;
    std::condition_variable scheduler_cv;
    SharedPlannerResult shared_result;
    GoalCandidate best_goal_candidate;
    std::string success_terminal_reason;

    std::atomic<bool> success_found{false};
    std::atomic<bool> shutdown{false};
    std::atomic<bool> deadline_hit{false};
    std::atomic<size_t> total_queue_entries{1};
    std::atomic<size_t> active_workers{0};
    std::atomic<size_t> total_queue_peak_size{1};
    std::atomic<uint64_t> insertion_order{1};
    std::atomic<size_t> global_expansion_index{0};

    const auto maybeSignalQuiescence = [&]() {
        if (!success_found.load() &&
            total_queue_entries.load() == 0 &&
            active_workers.load() == 0) {
            shutdown.store(true);
            scheduler_cv.notify_all();
        }
    };

    const auto recordHandoff = [&](size_t from_frontier_id, size_t to_frontier_id) {
        if (from_frontier_id == to_frontier_id) {
            return;
        }
        const uint64_t key =
            (static_cast<uint64_t>(from_frontier_id) << 32) |
            static_cast<uint64_t>(to_frontier_id);
        std::lock_guard<std::mutex> lock(handoff_mutex);
        auto& summary = handoff_summaries[key];
        summary.from_frontier_id = from_frontier_id;
        summary.to_frontier_id = to_frontier_id;
        ++summary.transfer_count;
        if (summary.first_transfer_ms < 0.0) {
            summary.first_transfer_ms =
                elapsedMilliseconds(search_loop_start_time, Clock::now());
        }
    };

    const auto considerGoalCandidate =
        [&](HybridAStarPlannerResult result,
            double path_cost,
            double required_straight_approach_m) {
            GoalCandidate candidate;
            candidate.result = std::move(result);
            candidate.path_cost = path_cost;
            candidate.required_straight_approach_m = required_straight_approach_m;
            candidate.terminal_straight_approach_m = computeTerminalStraightApproachLength(
                candidate.result.poses,
                candidate.result.segment_directions);
            candidate.valid = candidate.result.success;

            const bool strict_requirement_enabled =
                required_straight_approach_m > common::EPSILON;
            const bool meets_requirement =
                !strict_requirement_enabled ||
                candidate.terminal_straight_approach_m + 1e-6 >=
                    required_straight_approach_m;

            std::lock_guard<std::mutex> result_lock(result_mutex);
            if (meets_requirement) {
                if (!shared_result.published) {
                    shared_result.result = std::move(candidate.result);
                    shared_result.published = true;
                }
                return true;
            }

            if (shouldReplaceGoalCandidate(best_goal_candidate, candidate)) {
                best_goal_candidate = std::move(candidate);
            }
            return false;
        };

    const auto enqueueSuccessor =
        [&](size_t destination_frontier_id,
            SearchNode successor_node,
            const DiscreteStateKey& successor_key,
            double successor_h,
            double new_g) -> std::optional<size_t> {
        auto& frontier = *frontier_runtimes[destination_frontier_id];
        std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
        const auto existing = frontier.best_g_by_key.find(successor_key);
        if (existing != frontier.best_g_by_key.end() &&
            new_g >= existing->second - 1e-6) {
            return std::nullopt;
        }

        size_t successor_index = 0;
        {
            std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
            successor_index = nodes.size();
            nodes.push_back(std::move(successor_node));
        }

        frontier.best_g_by_key[successor_key] = new_g;
        const FrontierGuidanceContextRuntime* destination_guidance =
            destination_frontier_id < frontier_guidance_contexts.size()
                ? &frontier_guidance_contexts[destination_frontier_id]
                : nullptr;
        const double guidance_hint = computeFrontierGuidanceHint(
            costmap_bundle,
            grid_map::Position(successor_node.pose.x, successor_node.pose.y),
            destination_guidance);
        frontier.open_queue.push(OpenEntry{
            new_g + successor_h + (kInterfaceGuidanceTieBreakWeight * guidance_hint),
            new_g + successor_h,
            successor_h,
            new_g,
            guidance_hint,
            insertion_order.fetch_add(1),
            successor_index,
            successor_key
        });
        ++frontier.debug_summary.enqueued;
        if (frontier.debug_summary.first_enqueue_ms < 0.0) {
            frontier.debug_summary.first_enqueue_ms =
                elapsedMilliseconds(search_loop_start_time, Clock::now());
        }
        frontier.debug_summary.open_queue_peak_size = std::max(
            frontier.debug_summary.open_queue_peak_size,
            frontier.open_queue.size());
        total_queue_entries.fetch_add(1);
        total_queue_peak_size.store(std::max(
            total_queue_peak_size.load(),
            total_queue_entries.load()));
        scheduler_cv.notify_all();
        return successor_index;
    };

    struct AnalyticAttemptOutcome {
        HybridAStarPlannerResult result;
        PlannerAnalyticDebugEvent debug_event;
        double required_straight_approach_m{0.0};
        double path_cost{std::numeric_limits<double>::infinity()};
    };

    const auto worker = [&](size_t frontier_index) {
        FrontierRuntime& frontier = *frontier_runtimes[frontier_index];
        uint64_t local_expansion_count = 0;

        const auto tryAnalyticExpansion =
            [&](const SearchNode& node,
                size_t node_index,
                const std::string& trigger) -> AnalyticAttemptOutcome {
            common::ScopedProfilingTimer analytic_timer(
                &frontier.profiling_collector, "planner.analytic_expansion_attempt");
            AnalyticAttemptOutcome outcome;
            outcome.debug_event.attempted = true;
            outcome.debug_event.trigger = trigger;
            outcome.debug_event.distance_to_goal_m =
                std::hypot(request.goal.x - node.pose.x, request.goal.y - node.pose.y);
            if (!heuristic.hasOmplSpaces()) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "no_ompl_spaces";
                return outcome;
            }
            if (node.same_motion_remaining_to_change_m > common::EPSILON) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "same_motion_guard";
                return outcome;
            }

            const FrontierRuntime& node_frontier = *frontier_runtimes[node.frontier_id];
            const SearchContext& node_context = getSearchContext(node.behavior_name);
            const PlannerBehaviorProfile& node_profile =
                *node_context.profile;
            const HeuristicModel model =
                allowsReverseMotion(node_profile, node.zone)
                    ? HeuristicModel::REEDS_SHEPP
                    : HeuristicModel::DUBINS;

            std::pair<double, std::vector<std::array<double, 3>>> sampled_path;
            {
                common::ScopedProfilingTimer sample_path_timer(
                    &frontier.profiling_collector, "planner.analytic_sample_path");
                sampled_path = heuristic.samplePath(
                    model,
                    node.pose.x, node.pose.y, node.pose.theta.radians(),
                    request.goal.x, request.goal.y, request.goal.theta.radians(),
                    node_context.analytic_step_size);
            }
            auto& path_length = sampled_path.first;
            auto& waypoints = sampled_path.second;
            outcome.debug_event.path_length_m = path_length;
            outcome.debug_event.waypoint_count = waypoints.size();

            if (waypoints.empty() || path_length > node_context.analytic_max_length_m) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail =
                    waypoints.empty() ? "no_waypoints" : "path_length_exceeded";
                return outcome;
            }

            std::vector<ResolvedPlannerBehavior> wp_resolutions;
            wp_resolutions.reserve(waypoints.size());
            ResolvedPlannerBehavior current_resolution;
            current_resolution.frontier_id = node.frontier_id;
            current_resolution.zone = node.zone;
            current_resolution.behavior_name = node.behavior_name;
            current_resolution.profile = &node_profile;
            current_resolution.steady_behavior_name =
                node.active_transition.isActive()
                    ? node_frontier.descriptor.behavior_name
                    : node.behavior_name;
            current_resolution.active_transition = node.active_transition;
            current_resolution.transition_promotion_reason =
                transitionPromotionReasonName(node.transition_promotion_reason);
            wp_resolutions.push_back(current_resolution);
            for (size_t i = 1; i < waypoints.size(); ++i) {
                const auto& wp = waypoints[i];
                math::Pose2d wp_pose(wp[0], wp[1], math::Angle::from_radians(wp[2]));
                robot::RobotState rs;
                rs.x = wp[0];
                rs.y = wp[1];
                rs.yaw = wp[2];

                const grid_map::Position wp_pos(rs.x, rs.y);
                if (!costmap.isInside(wp_pos)) {
                    outcome.debug_event.outcome = "failed";
                    outcome.debug_event.detail = "waypoint_out_of_bounds";
                    return outcome;
                }

                const bool analytic_collision = [&]() {
                    return checkCompositeCollision(
                        wp_pose,
                        default_collision_config.lethal_threshold,
                        "planner.collision_check.analytic");
                }();
                if (analytic_collision) {
                    outcome.debug_event.outcome = "failed";
                    outcome.debug_event.detail = "waypoint_collision";
                    return outcome;
                }

                ResolvedPlannerBehavior resolved;
                {
                    common::ScopedProfilingTimer behavior_timer(
                        &frontier.profiling_collector, "planner.behavior_resolution");
                    resolved = PlannerBehaviorResolver::resolve(
                        wp_pose,
                        costmap_bundle.guidance_grid,
                        current_resolution.frontier_id,
                        current_resolution.zone,
                        current_resolution.behavior_name,
                        selection.frontiers,
                        behavior_set_,
                        current_resolution.active_transition);
                }
                if (resolved.profile == nullptr ||
                    resolved.frontier_id < current_resolution.frontier_id) {
                    outcome.debug_event.outcome = "failed";
                    outcome.debug_event.detail = "path_validation_failed";
                    return outcome;
                }
                current_resolution = resolved;
                wp_resolutions.push_back(resolved);
            }

            const AnalyticPathValidationResult analytic_validation =
                validateAnalyticPathSegments(
                    node,
                    waypoints,
                    wp_resolutions,
                    global_max_same_motion_length_m);
            if (!analytic_validation.valid) {
                outcome.debug_event.outcome = "failed";
                outcome.debug_event.detail = "path_validation_failed";
                return outcome;
            }

            HybridAStarPlannerResult result;
            result.success = true;
            {
                common::ScopedProfilingTimer reconstruction_timer(
                    &frontier.profiling_collector, "planner.result_reconstruction");
                std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
                std::vector<math::Pose2d> prefix_poses;
                std::vector<std::string> prefix_behaviors;
                std::vector<common::MotionDirection> prefix_directions;

                int idx = static_cast<int>(node_index);
                while (idx >= 0) {
                    const SearchNode& n = nodes[static_cast<size_t>(idx)];
                    prefix_poses.push_back(n.pose);
                    prefix_behaviors.push_back(n.behavior_name);
                    if (n.parent_index >= 0) {
                        prefix_directions.push_back(n.inbound_motion);
                    }
                    idx = n.parent_index;
                }

                std::reverse(prefix_poses.begin(), prefix_poses.end());
                std::reverse(prefix_behaviors.begin(), prefix_behaviors.end());
                std::reverse(prefix_directions.begin(), prefix_directions.end());

                for (size_t i = 1; i < waypoints.size(); ++i) {
                    prefix_poses.emplace_back(
                        waypoints[i][0],
                        waypoints[i][1],
                        math::Angle::from_radians(waypoints[i][2]));
                    prefix_behaviors.push_back(wp_resolutions[i].behavior_name);
                    prefix_directions.push_back(analytic_validation.segment_directions[i - 1]);
                }
                result.poses = std::move(prefix_poses);
                result.behavior_sequence = std::move(prefix_behaviors);
                result.segment_directions = std::move(prefix_directions);
            }
            result.detail = "Path found (analytic expansion).";
            outcome.required_straight_approach_m =
                wp_resolutions.back().profile != nullptr
                    ? wp_resolutions.back().profile->planner.min_goal_straight_approach_m
                    : 0.0;
            outcome.path_cost = node.g + path_length;
            outcome.result = std::move(result);
            outcome.debug_event.outcome = "success";
            return outcome;
        };

        while (true) {
            if (success_found.load() || shutdown.load()) {
                return;
            }
            if (Clock::now() >= deadline) {
                deadline_hit.store(true);
                shutdown.store(true);
                scheduler_cv.notify_all();
                return;
            }

            OpenEntry entry;
            bool have_entry = false;
            {
                std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
                if (!frontier.open_queue.empty()) {
                    entry = frontier.open_queue.top();
                    frontier.open_queue.pop();
                    total_queue_entries.fetch_sub(1);
                    ++frontier.counters.expansions_popped;
                    ++frontier.debug_summary.popped;
                    if (frontier.debug_summary.first_pop_ms < 0.0) {
                        frontier.debug_summary.first_pop_ms =
                            elapsedMilliseconds(search_loop_start_time, Clock::now());
                    }
                    const auto best_it = frontier.best_g_by_key.find(entry.state_key);
                    if (best_it == frontier.best_g_by_key.end() ||
                        entry.g > best_it->second + 1e-6) {
                        ++frontier.counters.stale_entries_skipped;
                        ++frontier.debug_summary.stale_entries_skipped;
                    } else {
                        have_entry = true;
                        active_workers.fetch_add(1);
                    }
                }
            }

            if (!have_entry) {
                maybeSignalQuiescence();
                if (shutdown.load() || success_found.load()) {
                    return;
                }
                std::unique_lock<std::mutex> wait_lock(scheduler_mutex);
                scheduler_cv.wait_for(wait_lock, std::chrono::milliseconds(2));
                continue;
            }

            SearchNode current_node;
            {
                std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
                current_node = nodes[entry.node_index];
            }
            const SearchContext& current_context =
                getSearchContext(current_node.behavior_name);
            const PlannerBehaviorProfile& current_profile = *current_context.profile;

            PlannerExpansionDebugEvent expansion_debug;
            if (debug_trace != nullptr) {
                expansion_debug.expansion_index =
                    global_expansion_index.fetch_add(1);
                expansion_debug.node_index = entry.node_index;
                expansion_debug.frontier_id = frontier.descriptor.frontier_id;
                expansion_debug.frontier_role =
                    frontierRoleName(frontier.descriptor.role);
                expansion_debug.pose = current_node.pose;
                expansion_debug.zone_name = zoneLabel(current_node.zone);
                expansion_debug.behavior_name = current_node.behavior_name;
                expansion_debug.g = current_node.g;
                expansion_debug.h = current_node.h;
                expansion_debug.f = entry.f;
                expansion_debug.distance_to_goal_m =
                    std::hypot(request.goal.x - current_node.pose.x,
                               request.goal.y - current_node.pose.y);
                {
                    std::lock_guard<std::mutex> frontier_lock(frontier.mutex);
                    expansion_debug.open_queue_size_after_pop = frontier.open_queue.size();
                }
                expansion_debug.transition_steady_behavior_name =
                    frontier.descriptor.behavior_name;
                expansion_debug.transition_promotion_reason =
                    transitionPromotionReasonName(
                        current_node.transition_promotion_reason);
                if (current_node.active_transition.isActive()) {
                    const grid_map::Position current_position(
                        current_node.pose.x,
                        current_node.pose.y);
                    expansion_debug.transition_entry_behavior_active = true;
                    expansion_debug.transition_entry_behavior_name =
                        current_node.active_transition.policy->entry_behavior;
                    expansion_debug.transition_steady_behavior_name =
                        frontier.descriptor.behavior_name;
                    expansion_debug.transition_entry_station_m =
                        current_node.active_transition.entry_station_m;
                    expansion_debug.transition_track_station_m = readRawLayerValue(
                        costmap,
                        costs::CostmapLayerNames::TRACK_STATION,
                        current_position);
                    expansion_debug.transition_lane_distance_m = readRawLayerValue(
                        costmap,
                        costs::CostmapLayerNames::LANE_DISTANCE,
                        current_position);
                    const double lane_heading = readRawLayerValue(
                        costmap,
                        costs::CostmapLayerNames::LANE_HEADING,
                        current_position);
                    if (std::isfinite(lane_heading)) {
                        expansion_debug.transition_lane_heading_error_rad =
                            std::abs(normalizeAngleSigned(
                                current_node.pose.theta.radians() - lane_heading));
                    }
                }
            }

            bool goal_satisfied = false;
            bool terminal_motion_valid = false;
            {
                common::ScopedProfilingTimer goal_check_timer(
                    &frontier.profiling_collector, "planner.goal_check");
                goal_satisfied = isGoalSatisfied(
                    current_node,
                    request.goal,
                    current_context.goal_distance_tolerance,
                    current_context.yaw_bin_size_rad);
                terminal_motion_valid = isTerminalMotionSegmentValid(current_node);
            }
            ++frontier.counters.goal_checks;
            if (debug_trace != nullptr) {
                expansion_debug.goal_satisfied = goal_satisfied;
                expansion_debug.terminal_motion_valid = terminal_motion_valid;
            }

            if (goal_satisfied && terminal_motion_valid) {
                ++frontier.counters.goal_hits;
                HybridAStarPlannerResult result;
                {
                    std::lock_guard<std::mutex> nodes_lock(nodes_mutex);
                    result = reconstructResult(
                        nodes,
                        static_cast<int>(entry.node_index),
                        &frontier.profiling_collector);
                }
                if (considerGoalCandidate(
                        std::move(result),
                        current_node.g,
                        current_profile.planner.min_goal_straight_approach_m)) {
                    success_terminal_reason = "goal_satisfied";
                    if (debug_trace != nullptr) {
                        frontier.expansion_events.push_back(std::move(expansion_debug));
                    }
                    success_found.store(true);
                    shutdown.store(true);
                    active_workers.fetch_sub(1);
                    scheduler_cv.notify_all();
                    return;
                }
            }

            ++local_expansion_count;
            const double distance_to_goal = std::hypot(
                request.goal.x - current_node.pose.x,
                request.goal.y - current_node.pose.y);
            const bool periodic_analytic_shot =
                current_profile.planner.analytic_shot &&
                local_expansion_count % current_context.analytic_expansion_interval == 0;
            const bool near_goal_analytic_shot =
                current_profile.planner.near_goal_analytic_expansion &&
                current_profile.planner.near_goal_analytic_radius_m > 0.0 &&
                distance_to_goal <= current_profile.planner.near_goal_analytic_radius_m;
            if (periodic_analytic_shot || near_goal_analytic_shot) {
                const std::string analytic_trigger =
                    periodic_analytic_shot && near_goal_analytic_shot
                        ? "periodic_and_near_goal"
                        : (periodic_analytic_shot ? "periodic" : "near_goal");
                const auto analytic_outcome =
                    tryAnalyticExpansion(current_node, entry.node_index, analytic_trigger);
                ++frontier.counters.analytic_attempts;
                if (debug_trace != nullptr) {
                    expansion_debug.analytic_attempted = true;
                    expansion_debug.analytic_event = analytic_outcome.debug_event;
                }
                if (analytic_outcome.result.success) {
                    if (considerGoalCandidate(
                            std::move(analytic_outcome.result),
                            analytic_outcome.path_cost,
                            analytic_outcome.required_straight_approach_m)) {
                        ++frontier.counters.analytic_successes;
                        success_terminal_reason = "analytic_expansion_success";
                        if (debug_trace != nullptr) {
                            frontier.expansion_events.push_back(std::move(expansion_debug));
                        }
                        success_found.store(true);
                        shutdown.store(true);
                        active_workers.fetch_sub(1);
                        scheduler_cv.notify_all();
                        return;
                    }
                }
                if (analytic_outcome.debug_event.detail == "no_ompl_spaces") {
                    ++frontier.counters.analytic_fail_no_ompl;
                } else if (analytic_outcome.debug_event.detail == "same_motion_guard") {
                    ++frontier.counters.analytic_fail_same_motion_guard;
                } else if (analytic_outcome.debug_event.detail == "path_length_exceeded" ||
                           analytic_outcome.debug_event.detail == "no_waypoints") {
                    ++frontier.counters.analytic_fail_path_length;
                } else if (analytic_outcome.debug_event.detail == "waypoint_out_of_bounds") {
                    ++frontier.counters.analytic_fail_out_of_bounds;
                } else if (analytic_outcome.debug_event.detail == "waypoint_collision") {
                    ++frontier.counters.analytic_fail_collision;
                } else {
                    ++frontier.counters.analytic_fail_validation;
                }
            }

            const auto projections = [&]() {
                common::ScopedProfilingTimer projections_timer(
                    &frontier.profiling_collector, "planner.motion_table_projections");
                return current_context.motion_table.getProjections(
                    static_cast<float>(current_node.pose.x / current_context.xy_resolution_m),
                    static_cast<float>(current_node.pose.y / current_context.xy_resolution_m),
                    current_node.heading_bin);
            }();

            bool lane_following_node = false;
            size_t forward_primitive_idx = projections.size();
            if (current_profile.planner.lane_primitive_suppression) {
                const grid_map::Position current_position(current_node.pose.x, current_node.pose.y);
                const auto current_lane_relaxation = computeTrackLaneRelaxation(
                    costmap_bundle,
                    frontier_track_lane_guidance,
                    current_node.frontier_id,
                    current_position);
                if (!current_lane_relaxation.inside_relax_window) {
                    const double current_lane_distance = readRawLayerValue(
                        costmap_bundle.guidance_grid,
                        costs::CostmapLayerNames::LANE_DISTANCE,
                        current_position);
                    const double current_lane_heading = readRawLayerValue(
                        costmap_bundle.guidance_grid,
                        costs::CostmapLayerNames::LANE_HEADING,
                        current_position);
                    if (std::isfinite(current_lane_distance) &&
                        std::isfinite(current_lane_heading) &&
                        current_lane_distance < current_profile.planner.step_size_m) {
                        const double heading_error = std::abs(normalizeAngleSigned(
                            current_node.pose.theta.radians() - current_lane_heading));
                        if (heading_error < (2.0 * current_context.yaw_bin_size_rad)) {
                            const auto forward_it = std::find_if(
                                projections.begin(),
                                projections.end(),
                                [](const auto& projection) {
                                    return projection.turn_dir == TurnDirection::FORWARD;
                                });
                            if (forward_it != projections.end()) {
                                lane_following_node = true;
                                forward_primitive_idx = static_cast<size_t>(
                                    std::distance(projections.begin(), forward_it));
                                ++frontier.counters.lane_following_candidates;
                                if (debug_trace != nullptr) {
                                    expansion_debug.lane_following_candidate = true;
                                }
                            }
                        }
                    }
                }
            }

            const auto recordPrimitiveEvent =
                [&](PlannerPrimitiveDebugEvent primitive_event) {
                    if (debug_trace == nullptr) {
                        return;
                    }
                    if (primitive_event.outcome == "out_of_bounds") {
                        ++frontier.counters.primitive_out_of_bounds;
                    } else if (primitive_event.outcome == "cross_track_pruned") {
                        ++frontier.counters.primitive_cross_track_pruned;
                    } else if (primitive_event.outcome == "behavior_unresolved") {
                        ++frontier.counters.primitive_behavior_unresolved;
                    } else if (primitive_event.outcome == "primitive_disallowed") {
                        ++frontier.counters.primitive_disallowed;
                    } else if (primitive_event.outcome == "collision") {
                        ++frontier.counters.primitive_collision;
                    } else if (primitive_event.outcome == "motion_change_blocked") {
                        ++frontier.counters.primitive_motion_change_blocked;
                    } else if (primitive_event.outcome == "dominated") {
                        ++frontier.counters.primitive_dominated;
                    } else if (primitive_event.outcome == "enqueued") {
                        ++frontier.counters.primitive_enqueued;
                    }
                    expansion_debug.primitive_events.push_back(std::move(primitive_event));
                };

            const auto hasForwardFrontierTransferCandidate =
                [&](size_t destination_frontier_id) {
                    for (const auto& candidate_projection : projections) {
                        if (motionDirectionForPrimitive(candidate_projection.turn_dir) !=
                            common::MotionDirection::Forward) {
                            continue;
                        }

                        const unsigned int candidate_heading_bin =
                            static_cast<unsigned int>(std::lround(candidate_projection.theta)) %
                            current_context.motion_table.getNumAngleBins();
                        math::Pose2d candidate_pose;
                        candidate_pose.x = static_cast<double>(candidate_projection.x) *
                            current_context.xy_resolution_m;
                        candidate_pose.y = static_cast<double>(candidate_projection.y) *
                            current_context.xy_resolution_m;
                        candidate_pose.theta = math::Angle::from_radians(
                            current_context.motion_table.getAngleFromBin(candidate_heading_bin));

                        const grid_map::Position candidate_position(
                            candidate_pose.x,
                            candidate_pose.y);
                        if (!costmap.isInside(candidate_position)) {
                            continue;
                        }

                        const ResolvedPlannerBehavior candidate_resolved =
                            PlannerBehaviorResolver::resolve(
                                candidate_pose,
                                costmap_bundle.guidance_grid,
                                current_node.frontier_id,
                                current_node.zone,
                                current_node.behavior_name,
                                selection.frontiers,
                                behavior_set_,
                                current_node.active_transition);
                        if (candidate_resolved.profile == nullptr ||
                            candidate_resolved.frontier_id != destination_frontier_id ||
                            !isPrimitiveAllowed(candidate_projection.turn_dir, candidate_resolved)) {
                            continue;
                        }
                        return true;
                    }
                    return false;
                };

            const auto tryExpandPrimitive = [&](size_t primitive_idx) -> bool {
                common::ScopedProfilingTimer primitive_timer(
                    &frontier.profiling_collector, "planner.primitive_expansion_attempt");
                const auto& projection = projections[primitive_idx];
                unsigned int successor_heading_bin =
                    static_cast<unsigned int>(std::lround(projection.theta)) %
                    current_context.motion_table.getNumAngleBins();
                const double travel_m =
                    static_cast<double>(current_context.motion_table.getTravelCost(
                        static_cast<unsigned int>(primitive_idx))) *
                    current_context.xy_resolution_m;
                double successor_travel_m = travel_m;

                math::Pose2d successor_pose;
                successor_pose.x = static_cast<double>(projection.x) *
                    current_context.xy_resolution_m;
                successor_pose.y = static_cast<double>(projection.y) *
                    current_context.xy_resolution_m;
                successor_pose.theta = math::Angle::from_radians(
                    current_context.motion_table.getAngleFromBin(successor_heading_bin));

                PlannerPrimitiveDebugEvent primitive_event;
                if (debug_trace != nullptr) {
                    primitive_event.primitive_index = primitive_idx;
                    primitive_event.turn_direction = turnDirectionName(projection.turn_dir);
                    primitive_event.successor_pose = successor_pose;
                    primitive_event.travel_m = travel_m;
                }

                grid_map::Position successor_position(successor_pose.x, successor_pose.y);
                if (!costmap.isInside(successor_position)) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "out_of_bounds";
                        primitive_event.detail = "successor_outside_costmap";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }

                ResolvedPlannerBehavior resolved = [&]() {
                    common::ScopedProfilingTimer behavior_timer(
                        &frontier.profiling_collector, "planner.behavior_resolution");
                    return PlannerBehaviorResolver::resolve(
                        successor_pose,
                        costmap_bundle.guidance_grid,
                        current_node.frontier_id,
                        current_node.zone,
                        current_node.behavior_name,
                        selection.frontiers,
                        behavior_set_,
                        current_node.active_transition);
                }();

                if (debug_trace != nullptr) {
                    primitive_event.resolved_behavior_name = resolved.behavior_name;
                    primitive_event.resolved_zone_name = zoneLabel(resolved.zone);
                }

                if (resolved.profile == nullptr) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "behavior_unresolved";
                        primitive_event.detail = "resolver_returned_null_profile";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
                if (resolved.frontier_id < current_node.frontier_id) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "primitive_disallowed";
                        primitive_event.detail = "backward_frontier_transfer_not_supported";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
                const bool primitive_allowed = isPrimitiveAllowed(projection.turn_dir, resolved);
                const bool reverse_track_handoff_override =
                    !primitive_allowed &&
                    motionDirectionForPrimitive(projection.turn_dir) ==
                        common::MotionDirection::Reverse &&
                    resolved.frontier_id != current_node.frontier_id &&
                    resolved.active_transition.isActive() &&
                    std::dynamic_pointer_cast<zones::TrackMainRoad>(resolved.zone) != nullptr &&
                    std::isfinite(resolved.transition_lane_heading_error_rad) &&
                    resolved.transition_lane_heading_error_rad >= (0.5 * common::PI) &&
                    !hasForwardFrontierTransferCandidate(resolved.frontier_id);
                if (!primitive_allowed && !reverse_track_handoff_override) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "primitive_disallowed";
                        primitive_event.detail = "primitive_not_allowed_for_behavior";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }

                if (resolved.frontier_id != current_node.frontier_id &&
                    resolved.active_transition.isActive()) {
                    const auto destination_track =
                        std::dynamic_pointer_cast<zones::TrackMainRoad>(resolved.zone);
                    if (destination_track != nullptr &&
                        current_node.zone != nullptr &&
                        resolved.frontier_id < frontier_track_lane_guidance.size() &&
                        frontier_track_lane_guidance[resolved.frontier_id].enabled) {
                        const auto snapped_pose = computeGoalDirectedTrackEntryPose(
                            current_node.zone,
                            destination_track,
                            request.goal,
                            frontier_track_lane_guidance[resolved.frontier_id].target_station_m,
                            resolved.active_transition.policy->min_depth_m);
                        if (!snapped_pose.has_value()) {
                            if (debug_trace != nullptr) {
                                primitive_event.outcome = "primitive_disallowed";
                                primitive_event.detail = "track_entry_alignment_unavailable";
                                recordPrimitiveEvent(std::move(primitive_event));
                            }
                            return false;
                        }
                        successor_pose = *snapped_pose;
                        successor_position = grid_map::Position(successor_pose.x, successor_pose.y);
                        successor_travel_m = std::hypot(
                            successor_pose.x - current_node.pose.x,
                            successor_pose.y - current_node.pose.y);
                        if (debug_trace != nullptr) {
                            primitive_event.successor_pose = successor_pose;
                            primitive_event.travel_m = successor_travel_m;
                        }
                    }
                }

                const auto successor_lane_relaxation = computeTrackLaneRelaxation(
                    costmap_bundle,
                    frontier_track_lane_guidance,
                    resolved.frontier_id,
                    successor_position);
                if (resolved.profile->planner.max_cross_track_error_m > 0.0 &&
                    !successor_lane_relaxation.inside_relax_window) {
                    const double lane_distance = readRawLayerValue(
                        costmap_bundle.guidance_grid,
                        costs::CostmapLayerNames::LANE_DISTANCE,
                        successor_position);
                    if (std::isfinite(lane_distance) &&
                        lane_distance > resolved.profile->planner.max_cross_track_error_m) {
                        if (debug_trace != nullptr) {
                            primitive_event.outcome = "cross_track_pruned";
                            primitive_event.detail = "lane_distance_exceeded";
                            recordPrimitiveEvent(std::move(primitive_event));
                        }
                        return false;
                    }
                }

                const float successor_lethal_threshold = static_cast<float>(
                    resolved.profile->collision_checker.lethal_threshold);
                if (std::abs(successor_lethal_threshold -
                             default_collision_config.lethal_threshold) > 0.5f) {
                    const bool in_collision = [&]() {
                        return checkCompositeCollision(
                            successor_pose,
                            successor_lethal_threshold,
                            "planner.collision_check.primitive_adjusted_threshold");
                    }();
                    if (in_collision) {
                        if (debug_trace != nullptr) {
                            primitive_event.outcome = "collision";
                            primitive_event.detail = "collision_with_adjusted_threshold";
                            recordPrimitiveEvent(std::move(primitive_event));
                        }
                        return false;
                    }
                } else {
                    const bool in_collision = [&]() {
                        return checkCompositeCollision(
                            successor_pose,
                            default_collision_config.lethal_threshold,
                            "planner.collision_check.primitive_default");
                    }();
                    if (in_collision) {
                        if (debug_trace != nullptr) {
                            primitive_event.outcome = "collision";
                            primitive_event.detail = "collision_with_default_threshold";
                            recordPrimitiveEvent(std::move(primitive_event));
                        }
                        return false;
                    }
                }

                const common::MotionDirection successor_motion =
                    motionDirectionForPrimitive(projection.turn_dir);
                if (!canChangeMotionDirection(current_node, successor_motion)) {
                    if (debug_trace != nullptr) {
                        primitive_event.outcome = "motion_change_blocked";
                        primitive_event.detail = "min_same_motion_length_not_satisfied";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }
                const double edge_cost = [&]() {
                    common::ScopedProfilingTimer edge_cost_timer(
                        &frontier.profiling_collector, "planner.edge_cost");
                    return computeEdgeCost(
                        costmap_bundle,
                        *resolved.profile,
                        current_node,
                        successor_position,
                        successor_lane_relaxation.bias_scale,
                        successor_pose.theta.radians(),
                        projection.turn_dir,
                        successor_travel_m);
                }();
                const double new_g = current_node.g + edge_cost;

                const double successor_same_motion_length_m =
                    computeNextSameMotionLength(
                        current_node,
                        successor_motion,
                        successor_travel_m,
                        global_max_same_motion_length_m);
                const double successor_same_motion_remaining_to_change_m =
                    computeNextSameMotionRemainingToChange(
                        current_node,
                        current_profile,
                        *resolved.profile,
                        successor_motion,
                        successor_travel_m);

                const SearchContext& destination_context =
                    getSearchContext(resolved.behavior_name);
                if (resolved.frontier_id != current_node.frontier_id ||
                    resolved.behavior_name != current_node.behavior_name) {
                    successor_heading_bin =
                        destination_context.motion_table.getClosestAngularBin(
                            normalizeAngleUnsigned(successor_pose.theta.radians()));
                }

                const double successor_h = computePlannerHeuristic(
                    costmap_bundle,
                    heuristic,
                    selectHeuristicModel(resolved),
                    successor_pose,
                    request.goal,
                    resolved.frontier_id < frontier_guidance_contexts.size()
                        ? &frontier_guidance_contexts[resolved.frontier_id]
                        : nullptr,
                    &frontier.profiling_collector);

                SearchNode successor_node;
                successor_node.pose = successor_pose;
                successor_node.frontier_id = resolved.frontier_id;
                successor_node.heading_bin = successor_heading_bin;
                successor_node.g = new_g;
                successor_node.h = successor_h;
                successor_node.parent_index = static_cast<int>(entry.node_index);
                successor_node.zone = resolved.zone;
                successor_node.behavior_name = resolved.behavior_name;
                successor_node.last_turn_class = classifyTurn(projection.turn_dir);
                successor_node.has_inbound_motion = true;
                successor_node.inbound_motion = successor_motion;
                successor_node.same_motion_length_m = successor_same_motion_length_m;
                successor_node.same_motion_remaining_to_change_m =
                    successor_same_motion_remaining_to_change_m;
                successor_node.active_transition = resolved.active_transition;
                successor_node.transition_promotion_reason =
                    resolved.transition_promotion_reason == "aligned_and_deep_enough"
                        ? TransitionPromotionReasonCode::AlignedAndDeepEnough
                        : (resolved.transition_promotion_reason == "max_depth_forced"
                               ? TransitionPromotionReasonCode::MaxDepthForced
                               : TransitionPromotionReasonCode::None);

                const DiscreteStateKey successor_key = discretizeState(
                    successor_pose,
                    successor_heading_bin,
                    successor_node,
                    destination_context.xy_resolution_m,
                    global_max_same_motion_length_m);
                const auto successor_index = enqueueSuccessor(
                    resolved.frontier_id,
                    std::move(successor_node),
                    successor_key,
                    successor_h,
                    new_g);
                if (!successor_index.has_value()) {
                    if (debug_trace != nullptr) {
                        primitive_event.edge_cost = edge_cost;
                        primitive_event.new_g = new_g;
                        primitive_event.successor_h = successor_h;
                        primitive_event.outcome = "dominated";
                        primitive_event.detail =
                            "existing_state_has_better_or_equal_cost";
                        recordPrimitiveEvent(std::move(primitive_event));
                    }
                    return false;
                }

                if (resolved.frontier_id != current_node.frontier_id) {
                    recordHandoff(current_node.frontier_id, resolved.frontier_id);
                }
                if (debug_trace != nullptr) {
                    primitive_event.edge_cost = edge_cost;
                    primitive_event.new_g = new_g;
                    primitive_event.successor_h = successor_h;
                    primitive_event.outcome = "enqueued";
                    primitive_event.detail =
                        resolved.frontier_id == current_node.frontier_id
                            ? "successor_added_to_frontier_queue"
                            : "successor_handed_off_to_next_frontier";
                    recordPrimitiveEvent(std::move(primitive_event));
                }
                return true;
            };

            if (lane_following_node &&
                forward_primitive_idx < projections.size() &&
                tryExpandPrimitive(forward_primitive_idx)) {
                ++frontier.counters.lane_suppression_forward_only_applied;
                if (debug_trace != nullptr) {
                    expansion_debug.lane_suppression_forward_success = true;
                    frontier.expansion_events.push_back(std::move(expansion_debug));
                }
                active_workers.fetch_sub(1);
                maybeSignalQuiescence();
                continue;
            }

            if (lane_following_node && forward_primitive_idx < projections.size()) {
                ++frontier.counters.lane_suppression_fallbacks;
                if (debug_trace != nullptr) {
                    expansion_debug.lane_suppression_fallback = true;
                }
            }

            for (size_t primitive_idx = 0; primitive_idx < projections.size(); ++primitive_idx) {
                tryExpandPrimitive(primitive_idx);
            }

            if (debug_trace != nullptr) {
                frontier.expansion_events.push_back(std::move(expansion_debug));
            }
            active_workers.fetch_sub(1);
            maybeSignalQuiescence();
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(frontier_runtimes.size());
    for (size_t frontier_index = 0; frontier_index < frontier_runtimes.size(); ++frontier_index) {
        workers.emplace_back(worker, frontier_index);
    }
    for (auto& worker_thread : workers) {
        worker_thread.join();
    }

    if (shared_result.published) {
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason = success_terminal_reason;
        }
        return finalizeResult(shared_result.result, nodes, total_queue_peak_size.load());
    }

    if (best_goal_candidate.valid) {
        if (deadline_hit.load()) {
            best_goal_candidate.result.detail = buildGoalCandidateFallbackDetail(
                best_goal_candidate,
                "timeout");
            if (debug_trace != nullptr) {
                debug_trace->terminal_reason =
                    "Returned best goal candidate after timeout because strict goal "
                    "straight-approach requirement was not met.";
            }
        } else {
            best_goal_candidate.result.detail = buildGoalCandidateFallbackDetail(
                best_goal_candidate,
                "frontier exhaustion");
            if (debug_trace != nullptr) {
                debug_trace->terminal_reason =
                    "Returned best goal candidate after frontier exhaustion because strict "
                    "goal straight-approach requirement was not met.";
            }
        }
        return finalizeResult(
            best_goal_candidate.result,
            nodes,
            total_queue_peak_size.load());
    }

    if (deadline_hit.load()) {
        if (debug_trace != nullptr) {
            debug_trace->terminal_reason =
                "Hybrid A* search timed out before finding a path.";
        }
        failure_result.detail = "Hybrid A* search timed out before finding a path.";
        return finalizeResult(failure_result, nodes, total_queue_peak_size.load());
    }

    if (debug_trace != nullptr) {
        debug_trace->terminal_reason =
            "Hybrid A* exhausted the frontier queues without finding a path.";
    }
    failure_result.detail =
        "Hybrid A* exhausted the frontier queues without finding a path.";
    return finalizeResult(failure_result, nodes, total_queue_peak_size.load());
}

HeuristicModel HybridAStarPlanner::selectHeuristicModel(
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (allowsReverseMotion(resolved_behavior)) {
        return HeuristicModel::REEDS_SHEPP;
    }
    return HeuristicModel::DUBINS;
}

bool HybridAStarPlanner::isPrimitiveAllowed(
    TurnDirection turn_direction,
    const ResolvedPlannerBehavior& resolved_behavior) {
    if (!allowsReverseMotion(resolved_behavior)) {
        return turn_direction != TurnDirection::REVERSE &&
               turn_direction != TurnDirection::REV_LEFT &&
               turn_direction != TurnDirection::REV_RIGHT;
    }
    return true;
}

} // namespace planning
} // namespace coastmotionplanning
