#include <iostream>
#include "DatasetInfo.hpp"

namespace sonarlog_target_tracking {

DatasetInfo::DatasetInfo()
{
}

DatasetInfo::DatasetInfo(const std::string& filename)
{
    Load(filename);
}

DatasetInfo::~DatasetInfo()
{
}

void DatasetInfo::Load(const std::string& filename)
{
    YAML::Node node = YAML::LoadFile(filename);
    assert(node.Type() == YAML::NodeType::Map);
    assert(node["log-entries"]);
    assert(node["log-entries"].Type() == YAML::NodeType::Sequence);

    // load log entries
    LoadLogEntries(node["log-entries"]);

    if (!node["training-settings"]) return;

    assert(node["training-settings"].Type() == YAML::NodeType::Map);

    // load training settings
    LoadTrainingSettings(node["training-settings"]);
}

void DatasetInfo::LoadLogEntries(const YAML::Node& node) {
    for (size_t i=0; i<node.size(); i++) {
        DatasetInfoEntry entry;
        NodeToEntry(node[i], entry);
        entries_.push_back(entry);
    }
}

void DatasetInfo::LoadTrainingSettings(const YAML::Node& node) {
    assert(node["model-filename"]);
    assert(node["output-directory"]);
    assert(node["hog-settings"]);
    assert(node["hog-settings"].Type() == YAML::NodeType::Map);
    assert(node["hog-settings"]["training-scale-factor"]);
    assert(node["hog-settings"]["show-descriptor"]);
    assert(node["hog-settings"]["window-size"]);
    assert(node["hog-settings"]["window-size"].Type() == YAML::NodeType::Map);
    assert(node["hog-settings"]["window-size"]["width"]);
    assert(node["hog-settings"]["window-size"]["height"]);

    training_settings_.model_filename = node["model-filename"].as<std::string>();
    training_settings_.output_directory = node["output-directory"].as<std::string>();
    training_settings_.hog_training_scale_factor = node["hog-settings"]["training-scale-factor"].as<double>();
    training_settings_.hog_show_descriptor = node["hog-settings"]["show-descriptor"].as<bool>();
    training_settings_.hog_window_size.width = node["hog-settings"]["window-size"]["width"].as<int>();
    training_settings_.hog_window_size.height = node["hog-settings"]["window-size"]["height"].as<int>();
}

void DatasetInfo::NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry) {
    assert(node["log-filename"]);
    assert(node["stream-name"]);
    entry.log_filename = node["log-filename"].as<std::string>();
    entry.stream_name = node["stream-name"].as<std::string>();
    entry.from_index = (node["from-index"]) ?  node["from-index"].as<int>() : -1;
    entry.to_index = (node["to-index"]) ?  node["to-index"].as<int>() : -1;
    entry.annotation_filename = (node["annotation-filename"]) ? node["annotation-filename"].as<std::string>() : "";
    entry.annotation_name = (node["annotation-name"]) ? node["annotation-name"].as<std::string>() : "";

    if (node["tranning-samples"]) {
        assert(node["tranning-samples"].Type() == YAML::NodeType::Sequence);
        YAML::Node training_samples_node = node["tranning-samples"];
        for (size_t i = 0; i < training_samples_node.size(); i++) {
            assert(training_samples_node[i]["from-sample"]);
            assert(training_samples_node[i]["to-sample"]);
            entry.training_intervals.push_back(
                SampleInterval(training_samples_node[i]["from-sample"].as<int>(),
                               training_samples_node[i]["to-sample"].as<int>()));
        }
    }
}

} /* namespace sonarlog_target_tracking */
