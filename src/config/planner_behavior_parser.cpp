#include "coastmotionplanning/config/planner_behavior_parser.hpp"

#include <stdexcept>
#include <unordered_set>

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

PlannerBehaviorProfiles PlannerBehaviorParser::parse(const std::string& master_params_filepath,
                                                     const std::string& behaviors_filepath) {
    const YAML::Node master_root = loadYamlFile(master_params_filepath, "master params");
    if (!master_root || !master_root.IsMap()) {
        throw std::runtime_error("Master params file must contain a YAML map: " + master_params_filepath);
    }

    const YAML::Node behaviors_root = loadYamlFile(behaviors_filepath, "planner behavior");
    const YAML::Node behaviors_node = behaviors_root["behaviors"];
    if (!behaviors_node || !behaviors_node.IsMap()) {
        throw std::runtime_error("Planner behavior file missing 'behaviors' map: " + behaviors_filepath);
    }

    PlannerBehaviorProfiles profiles;
    for (const auto& behavior_entry : behaviors_node) {
        const std::string profile_name = behavior_entry.first.as<std::string>("");
        if (profile_name.empty()) {
            throw std::runtime_error("Planner behavior names must be non-empty in: " + behaviors_filepath);
        }

        const YAML::Node profile_node = behavior_entry.second;
        validateProfileSchema(master_root, profile_node, profile_name, "");

        const YAML::Node active_layers = profile_node["active_layers"];
        if (!active_layers) {
            throw std::runtime_error("Behavior '" + profile_name + "' is missing required 'active_layers'.");
        }
        validateActiveLayers(active_layers, profile_name);

        profiles.emplace(profile_name, profile_node);
    }

    if (profiles.empty()) {
        throw std::runtime_error("Planner behavior file has no behavior profiles: " + behaviors_filepath);
    }

    return profiles;
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

} // namespace config
} // namespace coastmotionplanning
