#ifndef sonarlog_target_tracking_Common_hpp
#define sonarlog_target_tracking_Common_hpp

#include <string>
#include <vector>
#include <base/samples/Sonar.hpp>
#include <sonar_processing/HOGDetector.hpp>
#include <sonar_processing/ImageFiltering.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include <sonar_processing/Utils.hpp>
#include <rock_util/LogReader.hpp>
#include <rock_util/Utilities.hpp>
#include "DatasetInfo.hpp"

namespace sonarlog_target_tracking {

namespace common {

typedef void (*EXEC_SAMPLE_CALLBACK)(
    const base::samples::Sonar& sample,
    int sample_index,
    void *user_data);

inline void load_sonar_holder(const base::samples::Sonar& sample, sonar_processing::SonarHolder& sonar_holder, cv::Size sonar_size = cv::Size(-1, -1)) {
    sonar_holder.Reset(sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count,
        sonar_size);
}

void load_log_annotation(const std::string& logannotation_file, const std::string& annotation_name, std::vector<std::vector<cv::Point> >& annotation_points);

void load_samples(rock_util::LogStream& stream, std::vector<base::samples::Sonar>& samples, size_t total_samples=-1);

void load_samples_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    std::vector<base::samples::Sonar>& result_samples,
    std::vector<std::vector<cv::Point> >& result_annotations);

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

inline
sonar_processing::image_filtering::BorderFilterType border_filter_type(const std::string& type) {

    if (type == "scharr") {
        return sonar_processing::image_filtering::kSCharr;
    }

    if (type == "prewitt") {
        return sonar_processing::image_filtering::kPrewitt;
    }

    return sonar_processing::image_filtering::kSobel;
}

inline
sonar_processing::SonarImagePreprocessing::MeanDifferenceFilterSource mean_diff_filter_source(const std::string& type) {

    if (type == "border") {
        return sonar_processing::SonarImagePreprocessing::kBorder;
    }

    return sonar_processing::SonarImagePreprocessing::kEnhanced;
}

inline
void load_preprocessing_settings(
    const PreprocessingSettings& settings,
    sonar_processing::SonarImagePreprocessing& preprocessing) {

    preprocessing.set_roi_extract_thresh(settings.roi_extract_thresh);
    preprocessing.set_roi_extract_start_bin(settings.roi_extract_start_bin);
    preprocessing.set_mean_filter_ksize(settings.mean_filter_ksize);
    preprocessing.set_mean_difference_filter_enable(settings.mean_diff_filter_enable);
    preprocessing.set_mean_difference_filter_ksize(settings.mean_diff_filter_ksize);
    preprocessing.set_mean_difference_filter_source(mean_diff_filter_source(settings.mean_diff_filter_source));
    preprocessing.set_median_blur_filter_ksize(settings.median_blur_filter_ksize);
    preprocessing.set_border_filter_type(border_filter_type(settings.border_filter_type));
    preprocessing.set_border_filter_enable(settings.border_filter_enable);
    preprocessing.set_show_preprocessing_result(settings.show_preprocessing_result);
    preprocessing.set_enable_enhancement(settings.enable_enhancement);
    preprocessing.set_background_reducing_thresh(settings.background_reducing_thresh);
}

inline
void load_training_settings(
    const TrainingSettings& settings,
    sonar_processing::HOGDetector& detector) {
    detector.set_windown_size(settings.hog_window_size);
    detector.set_training_scale_factor(settings.hog_training_scale_factor);
    detector.set_show_descriptor(settings.hog_show_descriptor);
    detector.set_show_positive_window(settings.show_positive_window);
    detector.set_positive_input_validate(settings.positive_input_validate);
}

inline size_t find_best_weight_location_index(const std::vector<double>& weights)
{
    std::vector<size_t> indices(weights.size());
    for (size_t i = 0; i < indices.size(); i++) indices[i]=i;
    std::sort(indices.begin(), indices.end(), sonar_processing::utils::IndexComparator<double>(weights));
    std::reverse(indices.begin(), indices.end());
    return indices[0];
}


} /* namespace common */

} /* namespace sonarlog_target_tracking */

#endif
