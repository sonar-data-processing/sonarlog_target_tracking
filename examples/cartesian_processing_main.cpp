#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonarlog_annotation/AnnotationFileReader.hpp"
#include "base/test_config.h"
#include "cartesian_processing.hpp"

using namespace sonarlog_annotation;

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        // "/home/gustavoneves/masters_degree/dataset/logs/20160316-1127-06925_07750-gemini.0.log",
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.1.log",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.2.log",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.3.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    const std::string annotationfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0_annotation.yml",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.1_annotation.yml",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.2_annotation.yml",
        // DATA_PATH_STRING + "/logs/gemini-jequitaia.3_annotation.yml",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4_annotation.yml",
        DATA_PATH_STRING + "/logs/gemini-ferry.0_annotation.yml",
        DATA_PATH_STRING + "/logs/gemini-ferry.3_annotation.yml"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    sonar_processing::SonarHolder sonar_holder;
    std::vector<std::map<std::string, std::vector<cv::Point2f> > > annotations;

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        // AnnotationFileReader annotation_reader(annotationfiles[i]);
        // annotations = annotation_reader.read();

        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);


        int j = 0;
        do {
            sonar_holder.Reset(sample.bins,
                rock_util::Utilities::get_radians(sample.bearings),
                sample.beam_width.getRad(),
                sample.bin_count,
                sample.beam_count);

            int cart_line = 500;

            CartesianProcessing *cartesian_processing = NULL;

            if (!annotations.empty()) {
                cartesian_processing = new CartesianProcessing(sonar_holder, annotations[j]);
            }
            else {
                cartesian_processing = new CartesianProcessing(sonar_holder);
            }

            cartesian_processing->Process();

            if (stream.current_sample_index() == 1) cv::waitKey(); else cv::waitKey(25);
            stream.next<base::samples::Sonar>(sample);
            j++;

            delete cartesian_processing;

        } while(stream.current_sample_index() < stream.total_samples());
    }
}
