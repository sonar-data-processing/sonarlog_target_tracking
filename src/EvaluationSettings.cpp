#include <iostream>
#include "EvaluationSettings.hpp"

namespace sonarlog_target_tracking {

EvaluationSettings::EvaluationSettings()
{
}

EvaluationSettings::EvaluationSettings(const std::string& filename)
{
    Load(filename);
}

EvaluationSettings::~EvaluationSettings()
{
}

void EvaluationSettings::Load(const std::string& filename)
{
    YAML::Node node = YAML::LoadFile(filename);
    assert(node.Type() == YAML::NodeType::Map);
    LoadTrainingEvaluationSettings(node["training-evaluation-settings"]);
    LoadDetectionEvaluationSettings(node["detection-evaluation-settings"]);

}

void EvaluationSettings::LoadTrainingEvaluationSettings(const YAML::Node& node)
{
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Sequence);
    TrainingEvaluationSettings default_settings;
    NodeToTrainingEvaluationSettings(node[0], default_settings);
    training_evaluation_settings_.push_back(default_settings);
    for (size_t i = 1; i < node.size(); i++) {
        TrainingEvaluationSettings settings = default_settings;
        NodeToTrainingEvaluationSettings(node[i], settings);
        training_evaluation_settings_.push_back(settings);
    }
}

void EvaluationSettings::LoadDetectionEvaluationSettings(const YAML::Node& node)
{
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Sequence);
    DetectionEvaluationSettings default_settings;
    NodeToDetectionEvaluationSettings(node[0], default_settings);
    detection_evaluation_settings_.push_back(default_settings);
    for (size_t i = 1; i < node.size(); i++) {
        DetectionEvaluationSettings settings = default_settings;
        NodeToDetectionEvaluationSettings(node[i], settings);
        detection_evaluation_settings_.push_back(settings);
    }

}

void EvaluationSettings::NodeToTrainingEvaluationSettings(const YAML::Node& node, TrainingEvaluationSettings& settings)
{
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Map);

    if (node["border-filter-enable"]) {
      settings.border_filter_enable = node["border-filter-enable"].as<bool>();
    }

    if (node["border-filter-type"]) {
      settings.border_filter_type = node["border-filter-type"].as<std::string>();
    }

    if (node["mean-diff-filter-enable"]) {
      settings.mean_diff_filter_enable = node["mean-diff-filter-enable"].as<bool>();
    }

    if (node["positive-input-validate"]) {
      settings.positive_input_validate = node["positive-input-validate"].as<bool>();
    }

    if (node["training-scale-factor-begin"]) {
      settings.training_scale_factor_begin = node["training-scale-factor-begin"].as<double>();
    }

    if (node["training-scale-factor-end"]) {
      settings.training_scale_factor_end = node["training-scale-factor-end"].as<double>();
    }

    if (node["training-scale-factor-step"]) {
      settings.training_scale_factor_step = node["training-scale-factor-step"].as<double>();
    }

    if (node["window-size"] && node["window-size"].Type() == YAML::NodeType::Sequence) {
        settings.windows.clear();
        for (size_t i = 0; i < node["window-size"].size(); i++) {
            assert(node["window-size"][i]);
            assert(node["window-size"][i].Type() == YAML::NodeType::Map);
            assert(node["window-size"][i]["width"]);
            assert(node["window-size"][i]["height"]);
            int width = node["window-size"][i]["width"].as<int>();
            int height = node["window-size"][i]["height"].as<int>();
            settings.windows.push_back(cv::Size(width, height));
        }
    }
}

void EvaluationSettings::NodeToDetectionEvaluationSettings(const YAML::Node& node, DetectionEvaluationSettings& settings) {
    if (node["enable-location-best-weight-filter"]) {
        settings.enable_location_best_weight_filter = node["enable-location-best-weight-filter"].as<bool>();
    }

    if (node["detection-scale-factor-begin"]) {
        settings.detection_scale_factor_begin = node["detection-scale-factor-begin"].as<double>();
    }

    if (node["detection-scale-factor-end"]) {
        settings.detection_scale_factor_end = node["detection-scale-factor-end"].as<double>();
    }

    if (node["detection-scale-factor-step"]) {
        settings.detection_scale_factor_step = node["detection-scale-factor-step"].as<double>();
    }

    if (node["find-target-orientation-enable"]) {
        settings.find_target_orientation_enable = node["find-target-orientation-enable"].as<bool>();
    }

    if (node["find-target-orientation-steps"] &&
        node["find-target-orientation-steps"].Type() == YAML::NodeType::Sequence) {
        settings.find_target_orientation_steps.clear();
        for (size_t i = 0; i < node["find-target-orientation-steps"].size(); i++) {
            settings.find_target_orientation_steps.push_back(node["find-target-orientation-steps"][i].as<int>());
        }
    }

    if (node["hog-detector-settings"] &&
        node["hog-detector-settings"].Type() == YAML::NodeType::Map) {

        if (node["hog-detector-settings"]["scale-begin"]) {
            settings.hog_detector_scale_begin = node["hog-detector-settings"]["scale-begin"].as<double>();
        }

        if (node["hog-detector-settings"]["scale-end"]) {
            settings.hog_detector_scale_end = node["hog-detector-settings"]["scale-end"].as<double>();
        }

        if (node["hog-detector-settings"]["scale-step"]) {
            settings.hog_detector_scale_step = node["hog-detector-settings"]["scale-step"].as<double>();
        }

        if (node["hog-detector-settings"]["strides"] &&
            node["hog-detector-settings"]["strides"].Type() == YAML::NodeType::Sequence) {
            settings.hog_detector_strides.clear();
            for (size_t i = 0; i < node["hog-detector-settings"]["strides"].size(); i++) {
                settings.hog_detector_strides.push_back(node["hog-detector-settings"]["strides"][i].as<int>());
            }
        }
    }

}

} /* namespace sonarlog_target_tracking */
