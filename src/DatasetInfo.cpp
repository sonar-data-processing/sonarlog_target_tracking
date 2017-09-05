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

    // load preprocessing settings
    LoadPreprocessingSettings(node["preprocessing-settings"]);

    // load training settings
    LoadTrainingSettings(node["training-settings"]);

    LoadDetectionSettings(node["detection-settings"]);
}

void DatasetInfo::LoadLogEntries(const YAML::Node& node) {
    for (size_t i=0; i<node.size(); i++) {
        DatasetInfoEntry entry;
        NodeToEntry(node[i], entry);
        entries_.push_back(entry);
    }
}

void DatasetInfo::LoadPreprocessingSettings(const YAML::Node& node) {
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Map);

    if (node["roi-extract-thresh"]) {
        preprocessing_settings_.roi_extract_thresh = node["roi-extract-thresh"].as<double>();
    }

    if (node["roi-extract-start-bin"]) {
        preprocessing_settings_.roi_extract_start_bin  = node["roi-extract-start-bin"].as<int>();
    }

    if(node["border-filter-type"]) {
        preprocessing_settings_.border_filter_type = node["border-filter-type"].as<std::string>();
    }

    if(node["border-filter-enable"]) {
        preprocessing_settings_.border_filter_enable = node["border-filter-enable"].as<bool>();
    }

    if (node["mean-filter-ksize"]) {
        preprocessing_settings_.mean_filter_ksize = node["mean-filter-ksize"].as<int>();
    }

    if (node["mean-diff-filter-enable"]) {
        preprocessing_settings_.mean_diff_filter_enable = node["mean-diff-filter-enable"].as<bool>();
    }

    if (node["mean-diff-filter-ksize"]) {
        preprocessing_settings_.mean_diff_filter_ksize = node["mean-diff-filter-ksize"].as<int>();
    }

    if (node["mean-diff-filter-source"]) {
        preprocessing_settings_.mean_diff_filter_source = node["mean-diff-filter-source"].as<std::string>();
    }

    if (node["median-blur-filter-ksize"]) {
        preprocessing_settings_.median_blur_filter_ksize = node["median-blur-filter-ksize"].as<int>();
    }

    if (node["scale-factor"]) {
        preprocessing_settings_.scale_factor = node["scale-factor"].as<double>();
    }

    if (node["image-max-size"] && node["image-max-size"].Type() == YAML::NodeType::Map) {
        assert(node["image-max-size"]["width"]);
        assert(node["image-max-size"]["height"]);

        preprocessing_settings_.image_max_size.width = node["image-max-size"]["width"].as<int>();
        preprocessing_settings_.image_max_size.height = node["image-max-size"]["height"].as<int>();
    }
}

void DatasetInfo::LoadTrainingSettings(const YAML::Node& node) {
    assert(node["model-filename"]);
    assert(node["output-directory"]);
    assert(node["hog-settings"]);
    assert(node["hog-settings"].Type() == YAML::NodeType::Map);
    assert(node["hog-settings"]["training-scale-factor"]);
    assert(node["hog-settings"]["show-descriptor"]);
    assert(node["hog-settings"]["show-positive-window"]);
    assert(node["hog-settings"]["window-size"]);
    assert(node["hog-settings"]["window-size"].Type() == YAML::NodeType::Map);
    assert(node["hog-settings"]["window-size"]["width"]);
    assert(node["hog-settings"]["window-size"]["height"]);

    training_settings_.model_filename = node["model-filename"].as<std::string>();
    training_settings_.output_directory = node["output-directory"].as<std::string>();
    training_settings_.hog_training_scale_factor = node["hog-settings"]["training-scale-factor"].as<double>();
    training_settings_.hog_show_descriptor = node["hog-settings"]["show-descriptor"].as<bool>();
    training_settings_.show_positive_window = node["hog-settings"]["show-positive-window"].as<bool>();
    training_settings_.hog_window_size.width = node["hog-settings"]["window-size"]["width"].as<int>();
    training_settings_.hog_window_size.height = node["hog-settings"]["window-size"]["height"].as<int>();
}

void DatasetInfo::LoadDetectionSettings(const YAML::Node& node) {
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Map);

    if (node["evaluation-filename"]) {
        detection_settings_.evaluation_filename = node["evaluation-filename"].as<std::string>();
    }
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
