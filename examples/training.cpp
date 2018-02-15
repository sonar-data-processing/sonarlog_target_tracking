#include <iostream>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <rock_util/LogReader.hpp>
#include <sonar_processing/HOGDetector.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DatasetInfo.hpp"

void print_settings(const sonarlog_target_tracking::DatasetInfo& dataset_info) {
    std::cout << "Preprocessing Settings:" << std::endl;
    std::cout << dataset_info.preprocessing_settings().to_string() << std::endl;

    std::cout << "Training Settings:" << std::endl;
    std::cout << "full_model_filename: " << dataset_info.training_settings().full_model_filename() << std::endl;
    std::cout << dataset_info.training_settings().to_string() << std::endl;
}

int main(int argc, char **argv) {

    // sonarlog_target_tracking::ArgumentParser argument_parser;
    // if (!argument_parser.run(argc, argv)) {
    //     return -1;
    // }
    //
    // sonarlog_target_tracking::DatasetInfo dataset_info(argument_parser.dataset_info_filename());
    // std::vector<sonarlog_target_tracking::DatasetInfoEntry> positive_entries = dataset_info.positive_entries();
    // std::vector<sonarlog_target_tracking::DatasetInfoEntry> negative_entries = dataset_info.negative_entries();
    //
    // print_settings(dataset_info);
    //
    // sonar_processing::HOGDetector hog_detector;
    // sonar_processing::SonarImagePreprocessing preprocessing;
    //
    // sonarlog_target_tracking::common::load_training_settings(dataset_info.training_settings(), hog_detector);
    // sonarlog_target_tracking::common::load_preprocessing_settings(dataset_info.preprocessing_settings(), preprocessing);
    // preprocessing.set_background_reducing_thresh(0.0);
    //
    // hog_detector.set_sonar_image_processing(preprocessing);
    // hog_detector.set_sonar_image_size(dataset_info.preprocessing_settings().image_max_size);
    //
    //
    // std::vector<base::samples::Sonar> training_samples;
    // std::vector<std::vector<cv::Point> > training_annotations;
    //
    // for (size_t i=0; i<positive_entries.size(); i++) {
    //     sonarlog_target_tracking::common::load_training_data_from_dataset_entry(positive_entries[i], training_samples, training_annotations);
    // }
    //
    // std::vector<int> input_labels;
    // size_t j=0;
    // for (j=0; j<training_samples.size(); j++) input_labels.push_back(1);
    //
    // for (size_t i=0; i<negative_entries.size(); i++) {
    //     sonarlog_target_tracking::common::load_training_data_from_dataset_entry(negative_entries[i], training_samples, training_annotations);
    // }
    //
    // for (j; j<training_samples.size(); j++) input_labels.push_back(-1);
    //
    //
    // std::cout << "Total training samples: " << training_samples.size() << std::endl;
    // std::cout << "HOG Detector training..." << std::endl;
    // hog_detector.Train(training_samples, training_annotations, input_labels, dataset_info.training_settings().full_model_filename());
    // std::cout << "HOG Detector training finished" << std::endl;

    return 0;
}
