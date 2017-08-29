#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rock_util/LogReader.hpp>
#include <sonar_processing/HOGDetector.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/Utils.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DatasetInfo.hpp"
#include "DetectionEval.hpp"

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

struct Context {
    sonar_processing::HOGDetector hog_detector;
    sonar_processing::SonarHolder sonar_holder;
    std::vector<std::vector<cv::Point> > annotations;
    base::Time start_time;
    size_t sample_count;
    size_t detected_sample_count;
    size_t failed_sample_count;
    DetectionEvalList detection_eval_list;
};

// receive samples from sonar log reader
void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data) {
    Context *pContext = reinterpret_cast<Context*>(user_data);

    if (pContext->annotations.empty() || pContext->annotations[sample_index].empty()) {
        return;
    }

    pContext->sonar_holder.Reset(
        sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count,
        SonarHolder::LINEAR);

    std::vector<cv::RotatedRect> locations;
    std::vector<double> weights;

    bool result = pContext->hog_detector.Detect(
        pContext->sonar_holder.cart_image(),
        pContext->sonar_holder.cart_image_mask(),
        pContext->annotations[sample_index],
        locations,
        weights);

    cv::Mat output_image;
    pContext->sonar_holder.cart_image().copyTo(output_image);

    if (result) {
        image_util::draw_locations(output_image, locations, output_image);
        pContext->detected_sample_count++;
    }
    else {
       pContext->failed_sample_count++;
    }

    DetectionEval detection_eval(locations, pContext->annotations[sample_index], output_image.size());

    image_util::show_image("intersection_area_image", detection_eval.intersection_area_image(), 2);

    if (pContext->sample_count == 0) printf("\033[s");

    pContext->sample_count++;

    base::Time elapsed_time = base::Time::now()-pContext->start_time;
    double fps = pContext->sample_count/elapsed_time.toSeconds();
    printf("\033[uSample count: %d, Detected: %d, Failed: %d, Frame per seconds: %lf accuracy: %lf",
        pContext->sample_count,
        pContext->detected_sample_count,
        pContext->failed_sample_count,
        fps,
        detection_eval.accuracy());

    pContext->detection_eval_list.add_detection_evaluation(detection_eval);

    fflush(stdout);

    image_util::show_image("output", output_image, 2);
    cv::waitKey(15);
}

int main(int argc, char **argv) {

    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    sonarlog_target_tracking::DatasetInfo dataset_info(argument_parser.dataset_info_filename());
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = dataset_info.entries();

    Context context;

    context.hog_detector.set_windown_size(dataset_info.training_settings().hog_window_size);
    context.hog_detector.set_training_scale_factor(dataset_info.training_settings().hog_training_scale_factor);
    context.hog_detector.set_show_descriptor(dataset_info.training_settings().hog_show_descriptor);
    std::cout << "SVM Model Training: " << dataset_info.training_settings().full_model_filename() << std::endl;
    context.hog_detector.LoadSVMTrain(dataset_info.training_settings().full_model_filename());
    context.sample_count = 0;
    context.detected_sample_count = 0;
    context.failed_sample_count = 0;
    context.start_time = base::Time::now();

    for (size_t i=0; i<entries.size(); i++) {
        common::load_log_annotation(
            entries[i].annotation_filename,
            entries[i].annotation_name,
            context.annotations);

        printf("Processing log: %s\n", entries[i].log_filename.c_str());
        common::exec_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
        printf("\n");
    }
    printf("Save results into CSV file.\n");
    context.detection_eval_list.csv_write(dataset_info.training_settings().output_directory_path()+"/evaluation.csv");
    printf("Finished\n");

    return 0;
}
