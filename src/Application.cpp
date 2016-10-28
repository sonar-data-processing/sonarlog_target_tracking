#include <opencv2/opencv.hpp>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "sonar_util/Converter.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_util/Plot.hpp"
#include "sonar_processing/ImageUtils.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "sonarlog_target_tracking/Application.hpp"

using namespace sonar_processing;

namespace sonarlog_target_tracking {

Application *Application::instance_ = NULL;

Application*  Application::instance() {
    if (!instance_){
        instance_ = new Application();
    }
    return instance_;
}

void Application::init(const std::string& filename, const std::string& stream_name) {
    reader_.reset(new rock_util::LogReader(filename));
    plot_.reset(new base::Plot());
    stream_ = reader_->stream(stream_name);
}

void Application::process_next_sample() {
    base::samples::Sonar sample;
    stream_.next<base::samples::Sonar>(sample);
    std::vector<float> bearings_radians = rock_util::Utilities::get_radians(sample.bearings);
    TargetTrack target_track(sample.bins, bearings_radians, sample.beam_count, sample.bin_count);
    target_track.apply();
    cv::waitKey(100);
}

void Application::process_logfile() {
    uint32_t max_samples_log = 999;
    uint32_t sample_index = 0;
    stream_.reset();

    while (stream_.current_sample_index() < stream_.total_samples() && sample_index < max_samples_log) {
        process_next_sample();
        sample_index++;
    }
    cv::waitKey();
}

void Application::process_background_features_from_logfile(const std::string& filename, const std::string& stream_name) {
    rock_util::LogReader background_logfile(filename);
    rock_util::LogStream stream = background_logfile.stream(stream_name);

    double mean_sum = 0;
    double stddev_sum = 0;

    while (stream.current_sample_index() < stream.total_samples()) {
        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);
        // sonar_util::plot::polarshow("background data", sample.bins, rock_util::Utilities::get_radians(sample.bearings), sample.bin_count, sample.beam_count);
        cv::Mat src = cv::Mat(sample.bins).reshape(1, sample.beam_count);
        std::vector<double> bg_features = preprocessing::background_features_estimation(src, 9);
        mean_sum += bg_features[0];
        stddev_sum += bg_features[1];
    }

    background_features_.push_back(mean_sum / stream.total_samples());
    background_features_.push_back(stddev_sum / stream.total_samples());
}

void Application::plot(cv::Mat mat) {
    (*plot_)(image_utils::mat2vector<float>(mat));
}

}
