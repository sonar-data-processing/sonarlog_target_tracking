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
        hog_window_size = cv::Size(-1, -1);
    }

    TrainingSettings(
        const std::string& model_filename,
        const std::string& output_directory,
        double hog_training_scale_factor,
        bool hog_show_descriptor,
        cv::Size hog_window_size)
        : model_filename(model_filename)
        , output_directory(output_directory)
        , hog_training_scale_factor(hog_training_scale_factor)
        , hog_show_descriptor(false)
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
    cv::Size hog_window_size;
    std::string output_directory;
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

private:

    void LoadLogEntries(const YAML::Node& node);

    void LoadTrainingSettings(const YAML::Node& node);

    void NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry);

    void NodeToTrainingSettings();

    std::vector<DatasetInfoEntry> entries_;

    TrainingSettings training_settings_;

};

} /* namespace sonarlog_target_tracking */

#endif
