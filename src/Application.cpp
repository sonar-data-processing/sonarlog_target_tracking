#include <opencv2/opencv.hpp>
#include <base/samples/Sonar.hpp>
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonarlog_target_tracking/Application.hpp"

namespace sonarlog_target_tracking {

Application *Application::instance_ = NULL;

Application*  Application::instance() {
    if (!instance_){
        instance_ = new Application();
    }
    return instance_;
}

int Application::process_logfile(const std::string& filename, const std::string& stream_name) {

    rock_util::LogReader reader(filename);
    rock_util::LogStream stream = reader.stream(stream_name);
    base::samples::Sonar sample;

    while (stream.next<base::samples::Sonar>(sample)) {

        sonar_target_tracking::TargetTrack target_track(sample.bins,
                                                        rock_util::Utilities::get_radians(sample.bearings),
                                                        sample.beam_count, sample.bin_count);

        cv::Mat sonar_polar_image = rock_util::SonarSampleConverter::convert2polar(sample, 600);
        cv::imshow("sonar_polar_image", sonar_polar_image);
        cv::waitKey(200);
    }

    return 0;
}

}
