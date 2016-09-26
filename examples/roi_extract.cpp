#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_target_tracking/ImageUtils.hpp"
#include "sonar_target_tracking/SonarHolder.hpp"
#include "sonar_target_tracking/BasicOperations.hpp"
#include "base/test_config.h"

void roi_extract(const sonar_target_tracking::SonarHolder& holder) {
    sonar_target_tracking::basic_operations::horizontal_sum(holder);
}

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };
    
    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    sonar_target_tracking::SonarHolder sonar_holder;
    
    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");
        
        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);

        do {

            sonar_holder.Reset(sample.bins, 
                rock_util::Utilities::get_radians(sample.bearings),
                sample.beam_width.getRad(),
                sample.bin_count,
                sample.beam_count);
                stream.next<base::samples::Sonar>(sample);
            roi_extract(sonar_holder);
            break;
        } while(stream.current_sample_index() < stream.total_samples());
        break;
    }
}
