#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rock_util/LogReader.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DatasetInfo.hpp"

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

struct Context {
    sonar_processing::SonarHolder sonar_holder;
};

void perform_sonar_image_preprocessing(Context& context) {
    cv::Mat cart_image = context.sonar_holder.cart_image();
    cv::Mat preprocessed_image;
    cv::Mat preprocessed_mask;

    SonarImagePreprocessing sonar_image_preprocessing;
    sonar_image_preprocessing.Apply(context.sonar_holder, preprocessed_image, preprocessed_mask, 0.5);

    sonar_processing::image_util::show_image("cart_image", cart_image, 2);
    sonar_processing::image_util::show_image("preprocessed_image", preprocessed_image, 2);
    cv::waitKey(25);
}

void sample_receiver_callback(const base::samples::Sonar& sample, void *user_data) {
    Context *context = reinterpret_cast<Context*>(user_data);
    sonarlog_target_tracking::common::load_sonar_holder(sample, context->sonar_holder);
    perform_sonar_image_preprocessing(*context);
}

int main(int argc, char **argv) {

    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    sonarlog_target_tracking::DatasetInfo dataset_info(argument_parser.dataset_info_filename());
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = dataset_info.entries();

    Context context;

    for (size_t i=0; i<entries.size(); i++) {
        common::exec_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
    }

    return 0;
}
