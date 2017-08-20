#ifndef sonarlog_target_tracking_Common_hpp
#define sonarlog_target_tracking_Common_hpp

#include <string>
#include <vector>
#include <base/samples/Sonar.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <rock_util/LogReader.hpp>
#include <rock_util/Utilities.hpp>
#include "DatasetInfo.hpp"

namespace sonarlog_target_tracking {

namespace common {

template <typename T>
struct IndexComparator {
    IndexComparator(std::vector<T> vec) : vec_(vec) {}

    bool operator() (size_t i, size_t j) { return vec_[i]<vec_[j]; }

private:
    std::vector<T> vec_;
};

typedef void (*EXEC_SAMPLE_CALLBACK)(
    const base::samples::Sonar& sample,
    int sample_index,
    void *user_data);

inline void load_sonar_holder(const base::samples::Sonar& sample, sonar_processing::SonarHolder& sonar_holder) {
    sonar_holder.Reset(sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count);
}

void load_log_annotation(const std::string& logannotation_file, const std::string& annotation_name, std::vector<std::vector<cv::Point> >& annotation_points);

void load_samples(rock_util::LogStream& stream, std::vector<base::samples::Sonar>& samples, size_t total_samples=-1);

void adjust_annotation(
    cv::Size size,
    const std::vector<cv::Point>& src_points,
    std::vector<cv::Point>& dst_points,
    cv::Mat& annotation_mask);

bool file_exists(std::string filename);

void exec_samples(
    rock_util::LogStream& stream,
    int first_sample,
    int last_sample,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data = NULL);

void exec_samples_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data = NULL);

void exec_training_samples_from_dataset_entry (
    const DatasetInfoEntry& dataset_entry,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data = NULL);

void load_training_data_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    std::vector<base::samples::Sonar>& training_samples,
    std::vector<std::vector<cv::Point> >& training_annotations);

} /* namespace common */

} /* namespace sonarlog_target_tracking */

#endif
