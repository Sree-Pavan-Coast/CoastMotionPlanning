#include "coastmotionplanning/config/planner_behavior_parser.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "coastmotionplanning/zones/zone_type_utils.hpp"

namespace coastmotionplanning {
namespace config {

namespace {

std::string joinPath(const std::string& base, const std::string& key) {
    return base.empty() ? key : base + "." + key;
}

std::string nodeTypeName(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Undefined:
            return "undefined";
        case YAML::NodeType::Null:
            return "null";
        case YAML::NodeType::Scalar:
            return "scalar";
        case YAML::NodeType::Sequence:
            return "sequence";
        case YAML::NodeType::Map:
            return "map";
    }
    return "unknown";
}

YAML::Node loadYamlFile(const std::string& filepath, const std::string& label) {
    try {
        return YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load " + label + " YAML file: " + filepath + ". Error: " + e.what());
    }
}

} // namespace

PlannerBehaviorConfigFile PlannerBehaviorParser::parse(const std::string& behaviors_filepath) {
    const YAML::Node behaviors_root = loadYamlFile(behaviors_filepath, "planner behavior");
    if (!behaviors_root || !behaviors_root.IsMap()) {
        throw std::runtime_error(
            "Planner behavior file must contain a YAML map: " + behaviors_filepath);
    }

    for (const auto& root_entry : behaviors_root) {
        const std::string key = root_entry.first.as<std::string>("");
        if (key != "schema" && key != "global" && key != "behaviors" &&
            key != "transitions") {
            throw std::runtime_error(
                "Planner behavior file has unexpected top-level key '" + key + "'.");
        }
    }

    const YAML::Node schema_node = behaviors_root["schema"];
    if (!schema_node || !schema_node.IsMap()) {
        throw std::runtime_error("Planner behavior file missing required 'schema' map: " +
                                 behaviors_filepath);
    }

    const YAML::Node behaviors_node = behaviors_root["behaviors"];
    if (!behaviors_node || !behaviors_node.IsMap()) {
        throw std::runtime_error("Planner behavior file missing 'behaviors' map: " + behaviors_filepath);
    }

    PlannerBehaviorConfigFile config_file;
    config_file.global = parseGlobalConfig(behaviors_root);
    for (const auto& behavior_entry : behaviors_node) {
        const std::string profile_name = behavior_entry.first.as<std::string>("");
        if (profile_name.empty()) {
            throw std::runtime_error("Planner behavior names must be non-empty in: " + behaviors_filepath);
        }

        const YAML::Node profile_node = behavior_entry.second;
        validateProfileSchema(schema_node, profile_node, profile_name, "");

        const YAML::Node active_layers = profile_node["active_layers"];
        if (!active_layers) {
            throw std::runtime_error("Behavior '" + profile_name + "' is missing required 'active_layers'.");
        }
        validateActiveLayers(active_layers, profile_name);

        config_file.profiles.emplace(profile_name, parseProfile(profile_node));
    }

    if (config_file.profiles.empty()) {
        throw std::runtime_error("Planner behavior file has no behavior profiles: " + behaviors_filepath);
    }

    config_file.transition_policies =
        parseTransitionPolicies(behaviors_root, config_file.profiles);

    return config_file;
}

planning::PlannerBehaviorGlobalConfig PlannerBehaviorParser::parseGlobalConfig(
    const YAML::Node& behaviors_root) {
    planning::PlannerBehaviorGlobalConfig global_config;
    const YAML::Node global_node = behaviors_root["global"];
    if (!global_node) {
        return global_config;
    }
    if (!global_node.IsMap()) {
        throw std::runtime_error("Planner behavior file 'global' node must be a map.");
    }

    for (const auto& entry : global_node) {
        const std::string key = entry.first.as<std::string>("");
        if (key != "debug_mode" && key != "costmap_cache") {
            throw std::runtime_error(
                "Planner behavior file has unexpected global key '" + key + "'.");
        }
    }

    const YAML::Node debug_mode = global_node["debug_mode"];
    if (debug_mode) {
        if (!debug_mode.IsScalar()) {
            throw std::runtime_error(
                "Planner behavior file global.debug_mode must be a scalar.");
        }
        global_config.debug_mode = debug_mode.as<bool>();
    }

    const YAML::Node costmap_cache = global_node["costmap_cache"];
    if (costmap_cache) {
        if (!costmap_cache.IsMap()) {
            throw std::runtime_error(
                "Planner behavior file global.costmap_cache must be a map.");
        }

        for (const auto& entry : costmap_cache) {
            const std::string key = entry.first.as<std::string>("");
            if (key != "guidance_resolutions_m" &&
                key != "heuristic_resolutions_m" &&
                key != "dynamic_resolutions_m" &&
                key != "guidance_max_cells" &&
                key != "heuristic_max_cells" &&
                key != "dynamic_max_cells" &&
                key != "dynamic_window_size_x_m" &&
                key != "dynamic_window_size_y_m") {
                throw std::runtime_error(
                    "Planner behavior file has unexpected global.costmap_cache key '" + key + "'.");
            }
        }

        const auto parsePositiveResolutionSequence =
            [&](const char* key, std::vector<double>& output) {
                const YAML::Node node = costmap_cache[key];
                if (!node) {
                    return;
                }
                if (!node.IsSequence()) {
                    throw std::runtime_error(
                        std::string("Planner behavior file global.costmap_cache.") + key +
                        " must be a sequence.");
                }
                output.clear();
                for (const auto& entry : node) {
                    if (!entry.IsScalar()) {
                        throw std::runtime_error(
                            std::string("Planner behavior file global.costmap_cache.") + key +
                            " entries must be scalars.");
                    }
                    const double value = entry.as<double>();
                    if (!std::isfinite(value) || value <= 0.0) {
                        throw std::runtime_error(
                            std::string("Planner behavior file global.costmap_cache.") + key +
                            " entries must be finite and positive.");
                    }
                    output.push_back(value);
                }
            };

        parsePositiveResolutionSequence(
            "guidance_resolutions_m",
            global_config.costmap_resolution_policy.guidance_resolutions_m);
        parsePositiveResolutionSequence(
            "heuristic_resolutions_m",
            global_config.costmap_resolution_policy.heuristic_resolutions_m);
        parsePositiveResolutionSequence(
            "dynamic_resolutions_m",
            global_config.costmap_resolution_policy.dynamic_resolutions_m);

        const auto parsePositiveSizeT = [&](const char* key, size_t& output) {
            const YAML::Node node = costmap_cache[key];
            if (!node) {
                return;
            }
            if (!node.IsScalar()) {
                throw std::runtime_error(
                    std::string("Planner behavior file global.costmap_cache.") + key +
                    " must be a scalar.");
            }
            const auto value = node.as<long long>();
            if (value <= 0) {
                throw std::runtime_error(
                    std::string("Planner behavior file global.costmap_cache.") + key +
                    " must be positive.");
            }
            output = static_cast<size_t>(value);
        };
        parsePositiveSizeT(
            "guidance_max_cells",
            global_config.costmap_resolution_policy.guidance_max_cells);
        parsePositiveSizeT(
            "heuristic_max_cells",
            global_config.costmap_resolution_policy.heuristic_max_cells);
        parsePositiveSizeT(
            "dynamic_max_cells",
            global_config.costmap_resolution_policy.dynamic_max_cells);

        const auto parsePositiveDouble = [&](const char* key, double& output) {
            const YAML::Node node = costmap_cache[key];
            if (!node) {
                return;
            }
            if (!node.IsScalar()) {
                throw std::runtime_error(
                    std::string("Planner behavior file global.costmap_cache.") + key +
                    " must be a scalar.");
            }
            const double value = node.as<double>();
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error(
                    std::string("Planner behavior file global.costmap_cache.") + key +
                    " must be finite and positive.");
            }
            output = value;
        };
        parsePositiveDouble(
            "dynamic_window_size_x_m",
            global_config.costmap_resolution_policy.dynamic_window_size_x_m);
        parsePositiveDouble(
            "dynamic_window_size_y_m",
            global_config.costmap_resolution_policy.dynamic_window_size_y_m);
    }

    return global_config;
}

void PlannerBehaviorParser::validateProfileSchema(const YAML::Node& master_node,
                                                  const YAML::Node& profile_node,
                                                  const std::string& profile_name,
                                                  const std::string& current_path) {
    if (master_node.IsMap()) {
        if (!profile_node || !profile_node.IsMap()) {
            throw std::runtime_error(
                "Behavior '" + profile_name + "' must define map node '" + current_path +
                "', got " + nodeTypeName(profile_node) + ".");
        }

        for (const auto& master_entry : master_node) {
            const std::string key = master_entry.first.as<std::string>("");
            const std::string next_path = joinPath(current_path, key);
            if (!profile_node[key]) {
                throw std::runtime_error(
                    "Behavior '" + profile_name + "' is missing required key '" + next_path + "'.");
            }
            validateProfileSchema(master_entry.second, profile_node[key], profile_name, next_path);
        }

        for (const auto& profile_entry : profile_node) {
            const std::string key = profile_entry.first.as<std::string>("");
            if (!master_node[key]) {
                if (current_path.empty() && key == "active_layers") {
                    continue;
                }
                throw std::runtime_error(
                    "Behavior '" + profile_name + "' has unexpected key '" + joinPath(current_path, key) + "'.");
            }
        }
        return;
    }

    if (master_node.IsSequence()) {
        if (!profile_node || !profile_node.IsSequence()) {
            throw std::runtime_error(
                "Behavior '" + profile_name + "' must define sequence '" + current_path +
                "', got " + nodeTypeName(profile_node) + ".");
        }
        return;
    }

    if (!profile_node || !profile_node.IsScalar()) {
        throw std::runtime_error(
            "Behavior '" + profile_name + "' must define scalar '" + current_path +
            "', got " + nodeTypeName(profile_node) + ".");
    }
}

void PlannerBehaviorParser::validateActiveLayers(const YAML::Node& active_layers_node,
                                                 const std::string& profile_name) {
    if (!active_layers_node.IsSequence()) {
        throw std::runtime_error("Behavior '" + profile_name + "' must define 'active_layers' as a sequence.");
    }

    std::unordered_set<std::string> active_layers;
    for (const auto& layer_node : active_layers_node) {
        if (!layer_node.IsScalar()) {
            throw std::runtime_error(
                "Behavior '" + profile_name + "' has a non-scalar entry in 'active_layers'.");
        }

        const std::string layer_name = layer_node.as<std::string>("");
        if (layer_name.empty()) {
            throw std::runtime_error(
                "Behavior '" + profile_name + "' has an empty layer name in 'active_layers'.");
        }
        if (!active_layers.insert(layer_name).second) {
            throw std::runtime_error(
                "Behavior '" + profile_name + "' repeats active layer '" + layer_name + "'.");
        }
    }
}

planning::PlannerBehaviorProfile PlannerBehaviorParser::parseProfile(
    const YAML::Node& profile_node) {
    planning::PlannerBehaviorProfile profile;

    const YAML::Node planner = profile_node["planner"];
    profile.planner.max_planning_time_ms = planner["max_planning_time_ms"].as<int>();
    profile.planner.xy_grid_resolution_m = planner["xy_grid_resolution_m"].as<double>();
    profile.planner.yaw_grid_resolution_deg = planner["yaw_grid_resolution_deg"].as<double>();
    profile.planner.step_size_m = planner["step_size_m"].as<double>();
    profile.planner.only_forward_path = planner["only_forward_path"].as<bool>();
    profile.planner.weight_forward = planner["weight_forward"].as<double>();
    profile.planner.weight_reverse = planner["weight_reverse"].as<double>();
    profile.planner.weight_steer = planner["weight_steer"].as<double>();
    profile.planner.weight_steer_change = planner["weight_steer_change"].as<double>();
    profile.planner.weight_gear_change = planner["weight_gear_change"].as<double>();
    profile.planner.analytic_expansion_max_length_m =
        planner["analytic_expansion_max_length_m"].as<double>();
    profile.planner.analytic_expansion_ratio =
        planner["analytic_expansion_ratio"].as<double>();
    profile.planner.min_path_len_in_same_motion =
        planner["min_path_len_in_same_motion"].as<double>();
    profile.planner.min_goal_straight_approach_m =
        planner["min_goal_straight_approach_m"].as<double>();
    profile.planner.analytic_shot = planner["analytic_shot"].as<bool>();
    profile.planner.near_goal_analytic_expansion =
        planner["near_goal_analytic_expansion"].as<bool>();
    profile.planner.near_goal_analytic_radius_m =
        planner["near_goal_analytic_radius_m"].as<double>();
    profile.planner.weight_lane_centerline =
        planner["weight_lane_centerline"].as<double>();
    profile.planner.lane_heading_bias_weight =
        planner["lane_heading_bias_weight"].as<double>();
    profile.planner.max_cross_track_error_m =
        planner["max_cross_track_error_m"].as<double>();
    profile.planner.lane_primitive_suppression =
        planner["lane_primitive_suppression"].as<bool>();

    const YAML::Node costmap = profile_node["costmap"];
    profile.costmap.resolution_m = costmap["resolution_m"].as<double>();
    profile.costmap.inflation_radius_m = costmap["inflation_radius_m"].as<double>();
    profile.costmap.inscribed_radius_m = costmap["inscribed_radius_m"].as<double>();
    profile.costmap.cost_scaling_factor = costmap["cost_scaling_factor"].as<double>();
    profile.costmap.max_lane_cost = costmap["max_lane_cost"].as<double>();
    profile.costmap.max_lane_half_width_m = costmap["max_lane_half_width_m"].as<double>();

    const YAML::Node collision_checker = profile_node["collision_checker"];
    profile.collision_checker.collision_mode =
        collision_checker["collision_mode"].as<std::string>();
    profile.collision_checker.lethal_threshold =
        collision_checker["lethal_threshold"].as<double>();
    profile.collision_checker.margin_m = collision_checker["margin_m"].as<double>();

    const YAML::Node motion_primitives = profile_node["motion_primitives"];
    profile.motion_primitives.num_angle_bins =
        motion_primitives["num_angle_bins"].as<int>();

    const YAML::Node non_holonomic_heuristic = profile_node["non_holonomic_heuristic"];
    profile.non_holonomic_heuristic.lut_grid_size =
        non_holonomic_heuristic["lut_grid_size"].as<int>();
    profile.non_holonomic_heuristic.lut_cell_size_m =
        non_holonomic_heuristic["lut_cell_size_m"].as<double>();
    profile.non_holonomic_heuristic.hitch_angle_penalty_factor =
        non_holonomic_heuristic["hitch_angle_penalty_factor"].as<double>();

    for (const auto& layer_node : profile_node["active_layers"]) {
        profile.active_layers.insert(layer_node.as<std::string>());
    }

    return profile;
}

std::vector<planning::ZoneTypeTransitionPolicy> PlannerBehaviorParser::parseTransitionPolicies(
    const YAML::Node& behaviors_root,
    const PlannerBehaviorProfiles& profiles) {
    const YAML::Node transitions_node = behaviors_root["transitions"];
    if (!transitions_node) {
        return {};
    }
    if (!transitions_node.IsSequence()) {
        throw std::runtime_error(
            "Planner behavior file 'transitions' node must be a sequence.");
    }

    std::vector<planning::ZoneTypeTransitionPolicy> policies;
    policies.reserve(transitions_node.size());
    std::unordered_set<std::string> seen_pairs;

    for (size_t index = 0; index < transitions_node.size(); ++index) {
        const YAML::Node policy_node = transitions_node[index];
        const std::string label = "transitions[" + std::to_string(index) + "]";
        if (!policy_node.IsMap()) {
            throw std::runtime_error(label + " must be a map.");
        }

        for (const auto& entry : policy_node) {
            const std::string key = entry.first.as<std::string>("");
            if (key != "from_zone_type" &&
                key != "to_zone_type" &&
                key != "entry_behavior" &&
                key != "min_depth_m" &&
                key != "max_depth_m" &&
                key != "lane_distance_threshold_m" &&
                key != "heading_error_threshold_deg") {
                throw std::runtime_error(
                    label + " has unexpected key '" + key + "'.");
            }
        }

        const auto requireScalar = [&](const std::string& key) -> YAML::Node {
            const YAML::Node node = policy_node[key];
            if (!node || !node.IsScalar()) {
                throw std::runtime_error(label + "." + key + " must be a scalar.");
            }
            return node;
        };

        planning::ZoneTypeTransitionPolicy policy;
        policy.from_zone_type = requireScalar("from_zone_type").as<std::string>("");
        policy.to_zone_type = requireScalar("to_zone_type").as<std::string>("");
        policy.entry_behavior = requireScalar("entry_behavior").as<std::string>("");
        policy.min_depth_m = requireScalar("min_depth_m").as<double>();
        policy.max_depth_m = requireScalar("max_depth_m").as<double>();
        policy.lane_distance_threshold_m =
            requireScalar("lane_distance_threshold_m").as<double>();
        policy.heading_error_threshold_deg =
            requireScalar("heading_error_threshold_deg").as<double>();

        if (!zones::isSupportedZoneTypeName(policy.from_zone_type)) {
            throw std::runtime_error(
                label + ".from_zone_type '" + policy.from_zone_type +
                "' is not a supported zone type.");
        }
        if (!zones::isSupportedZoneTypeName(policy.to_zone_type)) {
            throw std::runtime_error(
                label + ".to_zone_type '" + policy.to_zone_type +
                "' is not a supported zone type.");
        }
        if (policy.entry_behavior.empty()) {
            throw std::runtime_error(label + ".entry_behavior must be non-empty.");
        }
        if (profiles.find(policy.entry_behavior) == profiles.end()) {
            throw std::runtime_error(
                label + ".entry_behavior '" + policy.entry_behavior +
                "' is not defined in behaviors.");
        }

        const auto ensureFiniteNonNegative = [&](double value, const std::string& key) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::runtime_error(
                    label + "." + key + " must be finite and non-negative.");
            }
        };

        ensureFiniteNonNegative(policy.min_depth_m, "min_depth_m");
        ensureFiniteNonNegative(policy.max_depth_m, "max_depth_m");
        ensureFiniteNonNegative(
            policy.lane_distance_threshold_m, "lane_distance_threshold_m");
        if (!std::isfinite(policy.heading_error_threshold_deg) ||
            policy.heading_error_threshold_deg <= 0.0) {
            throw std::runtime_error(
                label + ".heading_error_threshold_deg must be finite and positive.");
        }
        if (policy.max_depth_m < policy.min_depth_m) {
            throw std::runtime_error(
                label + ".max_depth_m must be greater than or equal to min_depth_m.");
        }

        const std::string pair_key = policy.from_zone_type + "->" + policy.to_zone_type;
        if (!seen_pairs.insert(pair_key).second) {
            throw std::runtime_error(
                "Planner behavior file repeats transition policy '" + pair_key + "'.");
        }

        policies.push_back(std::move(policy));
    }

    return policies;
}

} // namespace config
} // namespace coastmotionplanning
