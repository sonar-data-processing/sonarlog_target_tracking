#include <iostream>
#include <base/samples/Sonar.hpp>
#include <boost/filesystem.hpp>
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "base/test_config.h"

using namespace sonar_processing;

int main(int argc, char **argv) {

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        boost::filesystem::create_directories(boost::filesystem::path("output"));
        boost::filesystem::path p(logfiles[i]);

        base::samples::Sonar sample;
        while (stream.current_sample_index() < stream.total_samples()) {
            stream.next<base::samples::Sonar>(sample);
            cv::Mat src = cv::Mat(sample.bins).reshape(1, sample.beam_count);

            cv::Rect roi = preprocessing::calc_horiz_roi(src, 0.075);
            cv::Mat mat;
            src(roi).convertTo(mat, CV_8U, 255);

            preprocessing::remove_low_intensities_columns(mat, mat);

            cv::imshow("mat", mat);

            std::stringstream ss;
            ss << "output/" << p.stem().string() << "-" << stream.current_sample_index() << ".png";

            cv::imwrite(ss.str(), mat);
            cv::waitKey(100);
        }
    }

    return 0;
}
