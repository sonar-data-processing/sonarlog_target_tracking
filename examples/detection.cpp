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
    DatasetInfo dataset_info;
};

// receive samples from sonar log reader
void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data) {
    Context *pContext = reinterpret_cast<Context*>(user_data);

    if (pContext->annotations.empty() || pContext->annotations[sample_index].empty()) {
        return;
    }

    cv::Size sonar_image_size = pContext->dataset_info.preprocessing_settings().image_max_size;

    pContext->sonar_holder.Reset(
        sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count,
        sonar_image_size
     );

    std::vector<cv::RotatedRect> locations;
    std::vector<double> weights;

    std::vector<cv::Point> annotations = pContext->annotations[sample_index];

    if (sonar_image_size != cv::Size(-1, -1)) {
        image_util::resize_points(
            annotations,
            annotations,
            pContext->sonar_holder.cart_width_factor(),
            pContext->sonar_holder.cart_height_factor());
    }

    bool result = pContext->hog_detector.Detect(
        pContext->sonar_holder.cart_image(),
        pContext->sonar_holder.cart_image_mask(),
        annotations,
        locations,
        weights);

    cv::Mat output_image;
    pContext->sonar_holder.cart_image().copyTo(output_image);

    cv::Mat annotation_image;
    pContext->sonar_holder.cart_image().copyTo(annotation_image);

    image_util::draw_contour(annotation_image, annotation_image, cv::Scalar(0, 0, 255), annotations);
    cv::imshow("annotation_image", annotation_image);

    if (result) {
        image_util::draw_locations(output_image, locations, output_image);
        pContext->detected_sample_count++;
    }
    else {
       pContext->failed_sample_count++;
    }

    DetectionEval detection_eval(locations, annotations, output_image.size());

    cv::imshow("intersection_area_image", detection_eval.intersection_area_image());

    if (pContext->sample_count == 0) printf("\033[s");

    pContext->sample_count++;

    base::Time elapsed_time = base::Time::now()-pContext->start_time;
    double fps = pContext->sample_count/elapsed_time.toSeconds();
    printf("\033[uSample index: %d, Sample count: %d, Detected: %d, Failed: %d, Frame per seconds: %lf accuracy: %lf",
        sample_index,
        pContext->sample_count,
        pContext->detected_sample_count,
        pContext->failed_sample_count,
        fps,
        detection_eval.accuracy());

    pContext->detection_eval_list.add_detection_evaluation(detection_eval);

    fflush(stdout);

    cv::imshow("output", output_image);

    if (pContext->sample_count == 1) {
        cv::waitKey();
    }
    else {
        cv::waitKey(15);
    }
}

int main(int argc, char **argv) {

    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }




    Context context;
    context.dataset_info = DatasetInfo(argument_parser.dataset_info_filename());
    context.hog_detector.set_windown_size(context.dataset_info.training_settings().hog_window_size);
    context.hog_detector.set_training_scale_factor(context.dataset_info.training_settings().hog_training_scale_factor);
    context.hog_detector.set_show_descriptor(context.dataset_info.training_settings().hog_show_descriptor);

    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = context.dataset_info.entries();

    sonar_processing::SonarImagePreprocessing preprocessing;
    sonarlog_target_tracking::common::load_preprocessing_settings(context.dataset_info.preprocessing_settings(), preprocessing);
    context.hog_detector.set_sonar_image_processing(preprocessing);
    context.hog_detector.set_sonar_image_size(context.dataset_info.preprocessing_settings().image_max_size);

    std::cout << "SVM Model Training: " << context.dataset_info.training_settings().full_model_filename() << std::endl;
    context.hog_detector.LoadSVMTrain(context.dataset_info.training_settings().full_model_filename());

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
    // context.detection_eval_list.csv_write(dataset_info.training_settings().output_directory_path()+"/"+dataset_info.detection_settings().evaluation_filename);
    printf("Finished\n");

    return 0;
}
