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
    std::string log_filename;
    std::string stream_name;
    std::string annotation_filename;
    std::string annotation_name;
    int from_index;
    int to_index;

    std::vector<SampleInterval> training_intervals;

    std::string to_string() const {
        std::stringstream ss;
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
        hog_window_size = cv::Size(-1, -1);
    }

    TrainingSettings(
        const std::string& model_filename,
        const std::string& output_directory,
        double hog_training_scale_factor,
        bool hog_show_descriptor,
        bool show_positive_window,
        cv::Size hog_window_size)
        : model_filename(model_filename)
        , output_directory(output_directory)
        , hog_training_scale_factor(hog_training_scale_factor)
        , hog_show_descriptor(false)
        , show_positive_window(false)
        , hog_window_size(-1, -1)
    {
    }

    std::string to_string() const {
        std::stringstream ss;
        ss << "model_filename: " << model_filename << "\n";
        ss << "output_directory: " << output_directory << "\n";
        ss << "hog_training_scale_factor: " << hog_training_scale_factor << "\n";
        ss << "hog_show_descriptor: " << hog_show_descriptor << "\n";
        ss << "hog_window_size:\n";
        ss << "  - width: " << hog_window_size.width << "\n";
        ss << "  - height: " << hog_window_size.height << "\n";
        return ss.str();
    }

    std::string full_model_filename() const {
        return output_directory_path()+"/"+model_filename;
    }

    std::string output_directory_path() const {
        return output_directory.substr(0, output_directory.find_last_of("/"));
    }

    std::string model_filename;
    double hog_training_scale_factor;
    bool hog_show_descriptor;
    bool show_positive_window;
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
        mean_diff_filter_ksize = 15;
        median_blur_filter_ksize = 3;
        scale_factor = 0.5;
        border_filter_type  = "sobel";
        mean_diff_filter_source = "enhanced";
        border_filter_enable = true;
        image_max_size = cv::Size(-1, -1);
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
        const cv::Size& image_max_size)
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

    cv::Size image_max_size;
};

struct DetectionSettings {

    DetectionSettings()
    {
        evaluation_filename = "eval.csv";
    }

    DetectionSettings(
        const std::string& evaluation_filename)
        : evaluation_filename(evaluation_filename)
    {
    }

    std::string evaluation_filename;

};



class DatasetInfo
{

public:

    DatasetInfo();

    DatasetInfo(const std::string& filename);

    ~DatasetInfo();

    // load the dataset info YML file
    void Load(const std::string& filename);

    // returns dataset info entries
    const std::vector<DatasetInfoEntry>& entries() const {
        return entries_;
    }

    // returns training settings
    const TrainingSettings& training_settings() {
        return training_settings_;
    }

    // returns preprocessing settings
    const PreprocessingSettings& preprocessing_settings() {
        return preprocessing_settings_;
    }

    const DetectionSettings& detection_settings() {
        return detection_settings_;
    }

private:

    void LoadLogEntries(const YAML::Node& node);

    void LoadPreprocessingSettings(const YAML::Node& node);

    void LoadTrainingSettings(const YAML::Node& node);

    void LoadDetectionSettings(const YAML::Node& node);

    void NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry);

    void NodeToTrainingSettings();

    std::vector<DatasetInfoEntry> entries_;

    TrainingSettings training_settings_;
    PreprocessingSettings preprocessing_settings_;
    DetectionSettings detection_settings_;

};

} /* namespace sonarlog_target_tracking */

#endif
