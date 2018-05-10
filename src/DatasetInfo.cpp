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
    assert(node["log-entries-positive"]);
    assert(node["log-entries-positive"].Type() == YAML::NodeType::Sequence);

    // load positive log entries
    LoadLogEntries(node["log-entries-positive"], positive_entries_);

    // load negative log entries
    LoadLogEntries(node["log-entries-negative"], negative_entries_);

    // load extraction settings
    LoadExtractionSettings(node["extraction-settings"]);

    // load preprocessing settings
    LoadPreprocessingSettings(node["preprocessing-settings"]);

    if (!node["training-settings"]) return;

    assert(node["training-settings"].Type() == YAML::NodeType::Map);

    // load training settings
    LoadTrainingSettings(node["training-settings"]);

    // load detection settings
    LoadDetectionSettings(node["detection-settings"]);

}

void DatasetInfo::LoadLogEntries(const YAML::Node& node, std::vector<DatasetInfoEntry>& entries) {
    for (size_t i=0; i<node.size(); i++) {
        DatasetInfoEntry entry;
        NodeToEntry(node[i], entry);
        entries.push_back(entry);
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

    if (node["show-preprocessing-result"]) {
        preprocessing_settings_.show_preprocessing_result = node["show-preprocessing-result"].as<bool>();
    }

    if (node["enable-enhancement"]) {
        preprocessing_settings_.enable_enhancement = node["enable-enhancement"].as<bool>();
    }

    if (node["background-reducing-thresh"]) {
        preprocessing_settings_.background_reducing_thresh = node["background-reducing-thresh"].as<double>();
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

    if (node["positive-input-validate"]) {
        training_settings_.positive_input_validate = node["positive-input-validate"].as<bool>();
    }

}

void DatasetInfo::LoadDetectionSettings(const YAML::Node& node) {
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Map);

    if (node["evaluation-filename"]) {
        detection_settings_.evaluation_filename = node["evaluation-filename"].as<std::string>();
    }

    if (node["show-classifier-weights"]) {
        detection_settings_.show_classifier_weights = node["show-classifier-weights"].as<bool>();
    }

    if (node["enable-location-best-weight-filter"]) {
        detection_settings_.enable_location_best_weight_filter = node["enable-location-best-weight-filter"].as<bool>();
    }

    if (node["include-no-annotated-samples"]) {
        detection_settings_.include_no_annotated_samples = node["include-no-annotated-samples"].as<bool>();
    }

    if (node["detection-scale-factor"]) {
        detection_settings_.detection_scale_factor = node["detection-scale-factor"].as<double>();
    }

    if (node["detection-minimum-weight"]) {
        detection_settings_.detection_minimum_weight = node["detection-minimum-weight"].as<double>();
    }

    if (node["find-target-orientation-enable"]) {
        detection_settings_.find_target_orientation_enable = node["find-target-orientation-enable"].as<bool>();
    }

    if (node["find-target-orientation-step"]) {
        detection_settings_.find_target_orientation_step = node["find-target-orientation-step"].as<double>();
    }

    if (node["find-target-orientation-range"]) {
        detection_settings_.find_target_orientation_range = node["find-target-orientation-range"].as<double>();
    }

    if (node["overlap-threshold"]) {
        detection_settings_.overlap_threshold = node["overlap-threshold"].as<double>();
    }

    if (node["hog-detector-settings"] && node["hog-detector-settings"].Type() == YAML::NodeType::Map) {
        if (node["hog-detector-settings"]["positive-scale"]) {
            detection_settings_.hog_detector_positive_scale = node["hog-detector-settings"]["positive-scale"].as<double>();
        }
        else if (node["hog-detector-settings"]["scale"]) {
            detection_settings_.hog_detector_positive_scale = node["hog-detector-settings"]["scale"].as<double>();
        }

        if (node["hog-detector-settings"]["negative-scale"]) {
            detection_settings_.hog_detector_negative_scale = node["hog-detector-settings"]["negative-scale"].as<double>();
        }
        else {
            detection_settings_.hog_detector_negative_scale = detection_settings_.hog_detector_positive_scale;
        }

        if (node["hog-detector-settings"]["stride"]) {
            int stride = node["hog-detector-settings"]["stride"].as<int>();
            detection_settings_.hog_detector_stride = cv::Size(stride, stride);
        }
    }
}

void DatasetInfo::LoadExtractionSettings(const YAML::Node& node) {
    if (!node) return;
    assert(node.Type() == YAML::NodeType::Map);
    assert(node["class-id"]);
    assert(node["class-name"]);
    assert(node["extract-directory"]);

    extraction_settings_.class_id = node["class-id"].as<int>();
    extraction_settings_.class_name = node["class-name"].as<std::string>();
    extraction_settings_.extract_directory = node["extract-directory"].as<std::string>();


    if (node["extract-target-bbox"]) {
        extraction_settings_.extract_target_bbox = node["extract-target-bbox"].as<bool>();
    }

    if (node["extract-target-orientations"]) {
        extraction_settings_.extract_target_orientations = node["extract-target-orientations"].as<bool>();
    }

    if (node["extract-target-orientations-step"]) {
        extraction_settings_.extract_target_orientations_step = node["extract-target-orientations-step"].as<double>();
    }

    if (node["extract-target-orientations-keep"]) {
        extraction_settings_.extract_target_orientations_keep = node["extract-target-orientations-keep"].as<bool>();
    }

    if (node["extract-yolo-inputs"]) {
        extraction_settings_.extract_yolo_inputs = node["extract-yolo-inputs"].as<bool>();
    }

    if (node["extract-rotation-norm"]) {
        extraction_settings_.extract_rotation_norm = node["extract-rotation-norm"].as<bool>();
    }

    if (node["extract-annotation-mask"]) {
        extraction_settings_.extract_annotation_mask = node["extract-annotation-mask"].as<bool>();
    }

    if (node["extract-annotation-orientation"]) {
        extraction_settings_.extract_annotation_orientation = node["extract-annotation-orientation"].as<bool>();
    }

    if (node["save-source-image"]) {
        extraction_settings_.save_source_image = node["save-source-image"].as<bool>();
    }

    if (node["save-enhanced-image"]) {
        extraction_settings_.save_enhanced_image = node["save-enhanced-image"].as<bool>();
    }

    if (node["save-denoised-image"]) {
        extraction_settings_.save_denoised_image = node["save-denoised-image"].as<bool>();
    }

    if (node["save-preprocessed-image"]) {
        extraction_settings_.save_preprocessed_image = node["save-preprocessed-image"].as<bool>();
    }

    if (node["show-source-image"]) {
        extraction_settings_.show_source_image = node["show-source-image"].as<bool>();
    }

    if (node["show-enhanced-image"]) {
        extraction_settings_.show_enhanced_image = node["show-enhanced-image"].as<bool>();
    }

    if (node["show-denoised-image"]) {
        extraction_settings_.show_denoised_image = node["show-denoised-image"].as<bool>();
    }

    if (node["show-preprocessed-image"]) {
        extraction_settings_.show_preprocessed_image = node["show-preprocessed-image"].as<bool>();
    }

    if (node["show-annotation-image"]) {
        extraction_settings_.show_annotation_image = node["show-annotation-image"].as<bool>();
    }

    if (node["training-samples"]) {
       extraction_settings_.training_samples = node["training-samples"].as<bool>();
    }
}

void DatasetInfo::NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry) {
    assert(node["log-filename"]);
    assert(node["stream-name"]);
    entry.name = (node["name"]) ? node["name"].as<std::string>() : "";
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
