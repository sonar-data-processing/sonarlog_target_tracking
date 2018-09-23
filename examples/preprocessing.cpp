#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rock_util/LogReader.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include <rock_util/SonarSampleConverter.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/ImageFiltering.hpp>
#include <sonar_processing/Denoising.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DatasetInfo.hpp"

#include <stdio.h>

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

struct Context {
    DatasetInfo dataset_info;
    SonarHolder sonar_holder;
    SonarImagePreprocessing sonar_image_preprocessing;
};

void perform_sonar_image_preprocessing(Context& context) {
    cv::Mat source_image = context.sonar_holder.cart_image();
    cv::Mat source_mask = context.sonar_holder.cart_image_mask();
    cv::Mat preprocessed_image;
    cv::Mat preprocessed_mask;

    if (context.dataset_info.preprocessing_settings().image_max_size != cv::Size(-1, -1)) {
        cv::imshow("Source Image", source_image);

        context.sonar_image_preprocessing.Apply(
            source_image,
            source_mask,
            preprocessed_image,
            preprocessed_mask);
    }
    else {
        image_util::show_image("Source Image", source_image, 2);

        context.sonar_image_preprocessing.Apply(
            source_image,
            source_mask,
            preprocessed_image,
            preprocessed_mask,
            context.dataset_info.preprocessing_settings().scale_factor);
    }

    cv::waitKey(25);
}

// receive samples from sonar log reader
void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data) {
    Context *context = reinterpret_cast<Context*>(user_data);

    if (context->dataset_info.preprocessing_settings().image_max_size != cv::Size(-1, -1)) {
        sonarlog_target_tracking::common::load_sonar_holder(
            sample,
            context->sonar_holder,
            context->dataset_info.preprocessing_settings().image_max_size);
    }
    else {
        sonarlog_target_tracking::common::load_sonar_holder(sample, context->sonar_holder);
    }

    perform_sonar_image_preprocessing(*context);
}

int main(int argc, char **argv) {

    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    Context context;

    context.dataset_info =  DatasetInfo(argument_parser.dataset_info_filename());
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> positive_entries = context.dataset_info.positive_entries();
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> negative_entries = context.dataset_info.negative_entries();

    std::cout << context.dataset_info.preprocessing_settings().to_string() << std::endl;

    common::load_preprocessing_settings(
        context.dataset_info.preprocessing_settings(),
        context.sonar_image_preprocessing);

    for (size_t i=0; i<positive_entries.size(); i++) {
        common::exec_samples_from_dataset_entry(positive_entries[i], sample_receiver_callback, &context);
    }

    for (size_t i=0; i<negative_entries.size(); i++) {
        common::exec_samples_from_dataset_entry(negative_entries[i], sample_receiver_callback, &context);
    }

    return 0;
}
