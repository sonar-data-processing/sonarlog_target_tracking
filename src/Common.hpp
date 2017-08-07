#ifndef sonarlog_target_tracking_Common_hpp
#define sonarlog_target_tracking_Common_hpp

#include <string>
#include <vector>
#include <base/samples/Sonar.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <rock_util/LogReader.hpp>
#include <rock_util/Utilities.hpp>
#include "DetectionStats.hpp"
#include "DatasetInfo.hpp"

namespace sonarlog_target_tracking {

namespace common {

inline void load_sonar_holder(const base::samples::Sonar& sample, sonar_processing::SonarHolder& sonar_holder) {
    sonar_holder.Reset(sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count);
}

inline void save_detection_results(const std::string& filename,
                            const std::vector<std::vector<double> >& accuracy_levels,
                            const std::vector<std::vector<double> >& classifier_weights,
                            const std::vector<std::vector<cv::RotatedRect> >& detector_results,
                            const std::vector<cv::Size>& frame_sizes,
                            const std::vector<std::vector<cv::Point> >& annotations)
{
    DetectionStats stats(accuracy_levels,
                         classifier_weights,
                         detector_results,
                         frame_sizes,
                         annotations);
    stats.Save(filename);
    std::cout << "Save results in: " << filename << std::endl;
}


void load_log_annotation(const std::string& logannotation_file, const std::string& annotation_name, std::vector<std::vector<cv::Point> >& annotation_points);

void load_samples(rock_util::LogStream& stream, std::vector<base::samples::Sonar>& samples, size_t total_samples=-1);

void adjust_annotation(cv::Size size, const std::vector<cv::Point>& src_points, std::vector<cv::Point>& dst_points, cv::OutputArray annotation_mask);

bool file_exists(std::string filename);

void exec_samples(rock_util::LogStream& stream, int first_sample, int last_sample, void (*exec_samples_callback)(const base::samples::Sonar& sample, void *user_data), void *user_data = NULL);

void exec_samples_from_dataset_entry(const DatasetInfoEntry& dataset_entry, void (*exec_samples_callback)(const base::samples::Sonar& sample, void *user_data), void *user_data = NULL);



} /* namespace common */

} /* namespace sonarlog_target_tracking */

#endif
