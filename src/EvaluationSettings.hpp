#ifndef sonarlog_target_tracking_EvaluationSettings_hpp
#define sonarlog_target_tracking_EvaluationSettings_hpp

#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace sonarlog_target_tracking {

struct TrainingEvaluationSettings {
    TrainingEvaluationSettings()
        : border_filter_enable(true)
        , border_filter_type("sobel")
        , mean_diff_filter_enable(true)
        , positive_input_validate(true)
        , training_scale_factor_begin(0.25)
        , training_scale_factor_end(0.75)
        , training_scale_factor_step(0.25)
    {
        windows.push_back(cv::Size(32, 24));
        windows.push_back(cv::Size(56, 40));
    }

    std::string to_string() const
    {
        std::stringstream ss;
        ss << "border_filter_enable: " << border_filter_enable << "\n";
        ss << "border_filter_type: " << border_filter_type << "\n";
        ss << "mean_diff_filter_enable: " << mean_diff_filter_enable << "\n";
        ss << "positive_input_validate: " << positive_input_validate << "\n";
        ss << "training_scale_factor_begin: " << training_scale_factor_begin << "\n";
        ss << "training_scale_factor_end: " << training_scale_factor_end << "\n";
        ss << "training_scale_factor_step: " << training_scale_factor_step << "\n";
        ss << "windows: " << "\n";
        for (size_t i = 0; i < windows.size(); i++) {
            ss << "  width: " << windows[i].width << "\n";
            ss << "  height: " << windows[i].height << "\n";
        }
        return ss.str();
    }

    bool border_filter_enable;
    std::string border_filter_type;
    bool mean_diff_filter_enable;
    bool positive_input_validate;
    double training_scale_factor_begin;
    double training_scale_factor_end;
    double training_scale_factor_step;
    std::vector<cv::Size> windows;
};

struct DetectionEvaluationSettings {

    DetectionEvaluationSettings()
        : enable_location_best_weight_filter(true)
        , detection_scale_factor_begin(0.2)
        , detection_scale_factor_end(0.8)
        , detection_scale_factor_step(0.1)
        , find_target_orientation_enable(false)
        , hog_detector_scale_begin(1.025)
        , hog_detector_scale_end(1.5)
        , hog_detector_scale_step(0.025)
    {
        find_target_orientation_steps.push_back(15);
        find_target_orientation_steps.push_back(30);

        hog_detector_strides.push_back(4);
        hog_detector_strides.push_back(8);
        hog_detector_strides.push_back(16);
    }

    std::string to_string() const
    {
        std::stringstream ss;
        ss << "enable_location_best_weight_filter: " << enable_location_best_weight_filter << "\n";
        ss << "detection_scale_factor_begin: " << detection_scale_factor_begin << "\n";
        ss << "detection_scale_factor_end: " << detection_scale_factor_end << "\n";
        ss << "detection_scale_factor_step: " << detection_scale_factor_step << "\n";
        ss << "find_target_orientation_enable: " << find_target_orientation_enable << "\n";
        ss << "find_target_orientation_steps: ";
        for (size_t i = 0; i < find_target_orientation_steps.size(); i++) {
            ss <<  find_target_orientation_steps[i];
            if (i < find_target_orientation_steps.size()-1) ss << ", ";
        }
        ss << "\n";
        ss << "hog_detector_scale_begin: " << hog_detector_scale_begin << "\n";
        ss << "hog_detector_scale_end: " << hog_detector_scale_end << "\n";
        ss << "hog_detector_scale_step: " << hog_detector_scale_step << "\n";
        ss << "hog_detector_strides: ";
        for (size_t i = 0; i < hog_detector_strides.size(); i++) {
            ss <<  hog_detector_strides[i];
            if (i < hog_detector_strides.size()-1) ss << ", ";
        }
        ss << "\n";
        return ss.str();
    }

    bool enable_location_best_weight_filter;
    double detection_scale_factor_begin;
    double detection_scale_factor_end;
    double detection_scale_factor_step;
    bool find_target_orientation_enable;
    std::vector<int> find_target_orientation_steps;
    double hog_detector_scale_begin;
    double hog_detector_scale_end;
    double hog_detector_scale_step;
    std::vector<int> hog_detector_strides;
};

struct EvaluationHolder
{
    double frame_per_seconds_average;
    double accuracy_average;
    double precision_average;
    double recall_average;
    double fall_out_average;
    double overlap_region_average;
    double f1_score_average;
    size_t detected_sample_count;
    size_t failed_sample_count;
};

class EvaluationPersistent
{
public:

    enum {
        kSettings = 0,
        kFramePerSeconds,
        kAccuracy,
        kPrecision,
        kRecall,
        kFallout,
        kOverlapRegion,
        kF1Score,
        kDetected,
        kFailed,
        kPersistentFieldEnd
    };

    void open(const std::string& filename, std::map<std::string,EvaluationHolder>& map) {
        std::ifstream in(filename.c_str());
        std::string line;

        map.clear();

        while (std::getline(in, line)) {
            std::vector<std::string> cols;
            std::istringstream ss(line);
            std::string token;

            while (std::getline(ss, token, ',')) cols.push_back(token.c_str());
            assert(cols.size() == kPersistentFieldEnd);

            if (cols[kSettings] == "settings") continue;

            EvaluationHolder holder;
            holder.frame_per_seconds_average = atof(cols[kFramePerSeconds].c_str());
            holder.accuracy_average = atof(cols[kAccuracy].c_str());
            holder.precision_average = atof(cols[kPrecision].c_str());
            holder.recall_average = atof(cols[kRecall].c_str());
            holder.fall_out_average = atof(cols[kFallout].c_str());
            holder.overlap_region_average = atof(cols[kOverlapRegion].c_str());
            holder.f1_score_average = atof(cols[kF1Score].c_str());
            holder.detected_sample_count = strtoul(cols[kDetected].c_str(), NULL, 0);
            holder.failed_sample_count = strtoul(cols[kFailed].c_str(), NULL, 0);

            map.insert(std::make_pair(cols[kSettings], holder));
        }
    }

    void save(const std::string& filename, const std::map<std::string, EvaluationHolder>& map) {
        std::ofstream out;
        out.open(filename.c_str());
        out << "settings,frame-per-seconds,accuracy,precision,recall,fall-out,overlap-region,f1-score,detected,failed\n";
        std::map<std::string, EvaluationHolder>::const_iterator it = map.begin();

        while (it != map.end()) {
            out << it->first << ",";
            out << it->second.frame_per_seconds_average << ",";
            out << it->second.accuracy_average << ",";
            out << it->second.precision_average << ",";
            out << it->second.recall_average << ",";
            out << it->second.fall_out_average << ",";
            out << it->second.overlap_region_average << ",";
            out << it->second.f1_score_average << ",";
            out << it->second.detected_sample_count << ",";
            out << it->second.failed_sample_count << "\n";
            it++;
        }
        out.close();


        std::ifstream in(filename.c_str());
        std::ofstream out2((filename+".backup").c_str());
        out2 << in.rdbuf();
        out2.close();

    }
};

class EvaluationSettings
{
public:
    EvaluationSettings();

    EvaluationSettings(const std::string& filename);

    ~EvaluationSettings();

    // load the dataset info YML file
    void Load(const std::string& filename);

    const std::vector<TrainingEvaluationSettings>& training_evaluation_settings() {
        return training_evaluation_settings_;
    }

    const std::vector<DetectionEvaluationSettings>& detection_evaluation_settings() {
        return detection_evaluation_settings_;
    }

private:

    void LoadTrainingEvaluationSettings(const YAML::Node& node);

    void LoadDetectionEvaluationSettings(const YAML::Node& node);

    void NodeToTrainingEvaluationSettings(const YAML::Node& node, TrainingEvaluationSettings& settings);

    void NodeToDetectionEvaluationSettings(const YAML::Node& node, DetectionEvaluationSettings& settings);

    std::vector<TrainingEvaluationSettings> training_evaluation_settings_;
    std::vector<DetectionEvaluationSettings> detection_evaluation_settings_;

};

} /* namespace sonarlog_target_tracking */

#endif
