#ifndef sonarlog_target_tracking_DatasetInfo_hpp
#define sonarlog_target_tracking_DatasetInfo_hpp

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace sonarlog_target_tracking {

struct SampleInterval
{
    SampleInterval()
    {
        SampleInterval(-1, -1);
    }

    SampleInterval(int from, int to)
        : from(from)
        , to(to)
    {
    }

    int from;
    int to;
};

struct DatasetInfoEntry
{
    std::string name;
    std::string log_filename;
    std::string stream_name;
    std::string annotation_filename;
    std::string annotation_name;
    int from_index;
    int to_index;

    std::vector<SampleInterval> training_intervals;

    std::string to_string() const {
        std::stringstream ss;
        ss << "name: " << name << "\n";
        ss << "log_filename: " << log_filename << "\n";
        ss << "stream_name: " << stream_name << "\n";
        ss << "from_index: " << from_index << "\n";
        ss << "to_index: " << to_index << "\n";
        ss << "annotation_filename: " << ((annotation_filename.empty()) ? "empty" : annotation_filename) << "\n";
        ss << "annotation_name: " << ((annotation_name.empty()) ? "empty" : annotation_name) << "\n";
        ss << "training_intervals:\n";

        for (size_t i = 0; i < training_intervals.size(); i++) {
            ss << "- from: " << training_intervals[i].from;
            ss << "- to: " << training_intervals[i].to;
        }

        return ss.str();
    }
};

struct TrainingSettings {

    TrainingSettings()
    {
        model_filename = "";
        output_directory = "";
        hog_training_scale_factor = 1.0;
        hog_show_descriptor = false;
        show_positive_window = false;
        positive_input_validate = false;
        hog_window_size = cv::Size(-1, -1);
    }

    TrainingSettings(
        const std::string& model_filename,
        const std::string& output_directory,
        double hog_training_scale_factor,
        bool hog_show_descriptor,
        bool show_positive_window,
        bool positive_input_validate,
        cv::Size hog_window_size)
        : model_filename(model_filename)
        , output_directory(output_directory)
        , hog_training_scale_factor(hog_training_scale_factor)
        , hog_show_descriptor(false)
        , show_positive_window(false)
        , positive_input_validate(false)
        , hog_window_size(-1, -1)
    {
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "model_filename: " << model_filename << "\n";
        ss << "output_directory: " << output_directory << "\n";
        ss << "hog_training_scale_factor: " << hog_training_scale_factor << "\n";
        ss << "hog_show_descriptor: " << hog_show_descriptor << "\n";
        ss << "positive_input_validate: " << positive_input_validate << "\n";
        ss << "hog_window_size:\n";
        ss << "  - width: " << hog_window_size.width << "\n";
        ss << "  - height: " << hog_window_size.height << "\n";
        return ss.str();
    }

    std::string full_model_filename() const {
        return output_directory_path()+"/"+model_filename+"-"+get_window_size_string()+".yml";
    }

    std::string output_directory_path() const {
        return output_directory.substr(0, output_directory.find_last_of("/"));
    }

    std::string get_window_size_string() const {
        std::stringstream ss;
        ss << hog_window_size.width;
        ss << "x";
        ss << hog_window_size.height;
        return ss.str();
    }

    std::string model_filename;
    double hog_training_scale_factor;
    bool hog_show_descriptor;
    bool show_positive_window;
    bool positive_input_validate;
    cv::Size hog_window_size;
    std::string output_directory;
};

struct PreprocessingSettings {

    PreprocessingSettings()
    {
        roi_extract_thresh = 0.05;
        roi_extract_start_bin = 30;
        mean_filter_ksize = 5;
        mean_diff_filter_enable = true;
        mean_diff_filter_ksize = 100;
        median_blur_filter_ksize = 3;
        scale_factor = 0.5;
        border_filter_type  = "sobel";
        mean_diff_filter_source = "enhanced";
        border_filter_enable = true;
        image_max_size = cv::Size(-1, -1);
        show_preprocessing_result = false;
        enable_enhancement = true;
        background_reducing_thresh = 0.0;
    }

    PreprocessingSettings(
        double roi_extract_thresh,
        int roi_extract_start_bin,
        int mean_filter_ksize,
        bool mean_diff_filter_enable,
        int mean_diff_filter_ksize,
        std::string mean_diff_filter_source,
        int median_blur_filter_ksize,
        const std::string& border_filter_type,
        bool border_filter_enable,
        double scale_factor,
        const cv::Size& image_max_size,
        bool show_preprocessing_result,
        bool enable_enhancement,
        bool background_reducing_thresh)
        : roi_extract_thresh(roi_extract_thresh)
        , roi_extract_start_bin(roi_extract_start_bin)
        , mean_filter_ksize(mean_filter_ksize)
        , mean_diff_filter_enable(mean_diff_filter_enable)
        , mean_diff_filter_ksize(mean_diff_filter_ksize)
        , mean_diff_filter_source(mean_diff_filter_source)
        , median_blur_filter_ksize(median_blur_filter_ksize)
        , border_filter_type(border_filter_type)
        , border_filter_enable(border_filter_enable)
        , scale_factor(scale_factor)
        , image_max_size(image_max_size)
        , show_preprocessing_result(show_preprocessing_result)
        , enable_enhancement(enable_enhancement)
        , background_reducing_thresh(background_reducing_thresh)
    {
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "roi_extract_thresh: " << roi_extract_thresh << "\n";
        ss << "roi_extract_start_bin: " << roi_extract_start_bin << "\n";
        ss << "mean_filter_ksize: " << mean_filter_ksize << "\n";
        ss << "mean_diff_filter_ksize: " << mean_diff_filter_ksize << "\n";
        ss << "mean_diff_filter_enable: " << mean_diff_filter_enable << "\n";
        ss << "mean_diff_filter_source: " << mean_diff_filter_source << "\n";
        ss << "border_filter_type: " << border_filter_type << "\n";
        ss << "border_filter_enable: " << border_filter_enable << "\n";
        ss << "scale_factor: " << scale_factor << "\n";
        ss << "image_max_size: " << image_max_size << "\n";
        ss << "show_preprocessing_result: " << show_preprocessing_result << "\n";
        ss << "enable_enhancement: " << enable_enhancement << "\n";
        ss << "background_reducing_thresh: " << background_reducing_thresh << "\n";
        return ss.str();
    }

    double roi_extract_thresh;
    double scale_factor;

    int roi_extract_start_bin;
    int mean_filter_ksize;
    int mean_diff_filter_ksize;
    int median_blur_filter_ksize;

    std::string mean_diff_filter_source;
    std::string border_filter_type;

    bool mean_diff_filter_enable;
    bool border_filter_enable;
    bool show_preprocessing_result;
    bool enable_enhancement;

    double background_reducing_thresh;

    cv::Size image_max_size;
};

struct DetectionSettings {

    DetectionSettings()
    {
        evaluation_filename = "eval.csv";
        show_classifier_weights = false;
        enable_location_best_weight_filter = false;
        hog_detector_positive_scale = 1.5;
        hog_detector_negative_scale = 1.5;
        hog_detector_stride = cv::Size(8, 8);
        detection_scale_factor = 0.7;
        find_target_orientation_enable = false;
        find_target_orientation_step = true;
        find_target_orientation_range = 15;
        detection_minimum_weight = 0;
        include_no_annotated_samples = false;
        overlap_threshold = 0.3;
    }

    DetectionSettings(
        const std::string& evaluation_filename,
        bool show_classifier_weights,
        bool enable_location_best_weight_filter,
        double hog_detector_positive_scale,
        const cv::Size& hog_detector_stride,
        double detection_scale_factor,
        double detection_minimum_weight,
        bool find_target_orientation_enable,
        double find_target_orientation_step,
        double find_target_orientation_range,
        bool include_no_annotated_samples,
        double overlap_threshold)
        : evaluation_filename(evaluation_filename)
        , show_classifier_weights(show_classifier_weights)
        , enable_location_best_weight_filter(enable_location_best_weight_filter)
        , hog_detector_positive_scale(hog_detector_positive_scale)
        , hog_detector_negative_scale(hog_detector_negative_scale)
        , hog_detector_stride(hog_detector_stride)
        , detection_scale_factor(detection_scale_factor)
        , detection_minimum_weight(detection_minimum_weight)
        , find_target_orientation_enable(find_target_orientation_enable)
        , find_target_orientation_step(find_target_orientation_step)
        , find_target_orientation_range(find_target_orientation_range)
        , include_no_annotated_samples(include_no_annotated_samples)
        , overlap_threshold(overlap_threshold)
    {
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "evaluation_filename: " << evaluation_filename << "\n";
        ss << "show_classifier_weights: " << show_classifier_weights << "\n";
        ss << "enable_location_best_weight_filter: " << enable_location_best_weight_filter << "\n";
        ss << "hog_detector_positive_scale: " << hog_detector_positive_scale << "\n";
        ss << "hog_detector_negative_scale: " << hog_detector_negative_scale << "\n";
        ss << "hog_detector_stride: " << hog_detector_stride << "\n";
        ss << "detection_scale_factor: " << detection_scale_factor << "\n";
        ss << "detection_minimum_weight: " << detection_minimum_weight << "\n";
        ss << "find_target_orientation_enable: " << find_target_orientation_enable << "\n";
        ss << "find_target_orientation_step: " << find_target_orientation_step << "\n";
        ss << "find_target_orientation_range: " << find_target_orientation_range << "\n";
        ss << "include_no_annotated_samples: " << include_no_annotated_samples << "\n";
        ss << "overlap_threshold: " << overlap_threshold << "\n";
        return ss.str();
    }

    std::string evaluation_filename;

    bool show_classifier_weights;
    bool enable_location_best_weight_filter;
    bool include_no_annotated_samples;

    double hog_detector_positive_scale;
    double hog_detector_negative_scale;
    cv::Size hog_detector_stride;

    double detection_scale_factor;
    double detection_minimum_weight;
    bool find_target_orientation_enable;

    double find_target_orientation_step;
    double find_target_orientation_range;

    double overlap_threshold;
};
struct ExtractionSettings {

    ExtractionSettings()
    {
        class_id = 0;
        class_name = "";
        extract_target_bbox = false;
        extract_target_orientations = false;
        extract_target_orientations_step = 30;
        extract_target_orientations_keep = false;
        extract_directory = "";
        extract_yolo_inputs = true;
        extract_rotation_norm = false;
        extract_annotation_mask = false;
        extract_annotation_orientation = false;
        extract_target_orientations_keep = false;
        save_source_image = true;
        save_enhanced_image = true;
        save_denoised_image = true;
        save_preprocessed_image = true;
        show_source_image = false;
        show_enhanced_image = false;
        show_denoised_image = false;
        show_preprocessed_image = false;
        show_annotation_image = false;
        training_samples = false;
    }

    ExtractionSettings(
        int class_id,
        const std::string& class_name,
        const std::string& extract_directory,
        bool extract_target_bbox,
        bool extract_target_orientations,
        double extract_target_orientations_step,
        bool extract_target_orientations_keep,
        bool extract_yolo_inputs,
        bool extract_rotation_norm,
        bool extract_annotation_mask,
        bool extract_annotation_orientation,
        bool save_source_image,
        bool save_enhanced_image,
        bool save_denoised_image,
        bool save_preprocessed_image,
        bool show_source_image,
        bool show_enhanced_image,
        bool show_denoised_image,
        bool show_preprocessed_image,
        bool show_annotation_image,
        bool training_samples)
        : class_id(class_id)
        , class_name(class_name)
        , extract_target_bbox(extract_target_bbox)
        , extract_target_orientations(extract_target_orientations)
        , extract_target_orientations_step(extract_target_orientations_step)
        , extract_target_orientations_keep(extract_target_orientations_keep)
        , extract_directory(extract_directory)
        , extract_yolo_inputs(extract_yolo_inputs)
        , extract_rotation_norm(extract_rotation_norm)
        , extract_annotation_mask(extract_annotation_mask)
        , extract_annotation_orientation(extract_annotation_orientation)
        , save_source_image(save_source_image)
        , save_enhanced_image(save_enhanced_image)
        , save_denoised_image(save_denoised_image)
        , save_preprocessed_image(save_preprocessed_image)
        , show_source_image(show_source_image)
        , show_enhanced_image(show_enhanced_image)
        , show_denoised_image(show_denoised_image)
        , show_preprocessed_image(show_preprocessed_image)
        , show_annotation_image(show_annotation_image)
        , training_samples(training_samples)
    {
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "class_id: " << class_id << "\n";
        ss << "class_name: " << class_name << "\n";
        ss << "extract_target_bbox: " << extract_target_bbox << "\n";
        ss << "extract_target_orientations: " << extract_target_orientations << "\n";
        ss << "extract_target_orientations_step: " << extract_target_orientations_step << "\n";
        ss << "extract_target_orientations_keep: " << extract_target_orientations_keep << "\n";
        ss << "extract_directory: " << extract_directory << "\n";
        ss << "extract_yolo_inputs: " << extract_yolo_inputs << "\n";
        ss << "extract_rotation_norm: " << extract_rotation_norm << "\n";
        ss << "extract_annotation_mask: " << extract_annotation_mask << "\n";
        ss << "extract_annotation_orientation: " << extract_annotation_orientation << "\n";
        ss << "save_source_image: " << save_source_image << "\n";
        ss << "save_enhanced_image: " << save_enhanced_image << "\n";
        ss << "save_denoised_image: " << save_denoised_image << "\n";
        ss << "save_preprocessed_image: " << save_preprocessed_image << "\n";
        ss << "show_source_image: " << show_source_image << "\n";
        ss << "show_enhanced_image: " << show_enhanced_image << "\n";
        ss << "show_denoised_image: " << show_denoised_image << "\n";
        ss << "show_preprocessed_image: " << show_preprocessed_image << "\n";
        ss << "show_annotation_image: " << show_annotation_image << "\n";
        ss << "training_samples: " << training_samples << "\n";
        return ss.str();
    }

    int class_id;
    std::string class_name;
    std::string extract_directory;
    bool extract_target_bbox;
    bool extract_target_orientations;
    double extract_target_orientations_step;
    bool extract_yolo_inputs;
    bool extract_rotation_norm;
    bool extract_annotation_mask;
    bool extract_annotation_orientation;
    bool extract_target_orientations_keep;
    bool save_source_image;
    bool save_enhanced_image;
    bool save_denoised_image;
    bool save_preprocessed_image;
    bool show_source_image;
    bool show_enhanced_image;
    bool show_denoised_image;
    bool show_preprocessed_image;
    bool show_annotation_image;
    bool training_samples;
};




class DatasetInfo
{

public:

    DatasetInfo();

    DatasetInfo(const std::string& filename);

    ~DatasetInfo();

    // load the dataset info YML file
    void Load(const std::string& filename);

    // returns dataset info positive entries
    const std::vector<DatasetInfoEntry>& positive_entries() const {
        return positive_entries_;
    }

    // returns dataset info negative entries
    const std::vector<DatasetInfoEntry>& negative_entries() const {
        return negative_entries_;
    }

    // returns training settings
    TrainingSettings training_settings() const {
        return training_settings_;
    }

    // returns preprocessing settings
    PreprocessingSettings preprocessing_settings() const {
        return preprocessing_settings_;
    }

    // returns detection settings
    DetectionSettings detection_settings() const {
        return detection_settings_;
    }

    // returns extraction settings
    ExtractionSettings extraction_settings() const {
        return extraction_settings_;
    }

private:

    void LoadLogEntries(const YAML::Node& node, std::vector<DatasetInfoEntry>& entries);

    void LoadPreprocessingSettings(const YAML::Node& node);

    void LoadTrainingSettings(const YAML::Node& node);

    void LoadDetectionSettings(const YAML::Node& node);

    void LoadExtractionSettings(const YAML::Node& node);

    void NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry);

    void NodeToTrainingSettings();

    std::vector<DatasetInfoEntry> positive_entries_;
    std::vector<DatasetInfoEntry> negative_entries_;

    TrainingSettings training_settings_;
    PreprocessingSettings preprocessing_settings_;
    DetectionSettings detection_settings_;
    ExtractionSettings extraction_settings_;

};

} /* namespace sonarlog_target_tracking */

#endif
