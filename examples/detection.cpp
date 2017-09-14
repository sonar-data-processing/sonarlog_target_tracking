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

    double precision_sum;
    double accuracy_sum;
    double recall_sum;
    double fall_out_sum;
    double overlap_region_sum;
    double f1_score_sum;
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

    bool result;
    if (pContext->dataset_info.detection_settings().find_target_orientation_enable) {
        result = pContext->hog_detector.Detect(
            pContext->sonar_holder.cart_image(),
            pContext->sonar_holder.cart_image_mask(),
            locations,
            weights);
    }
    else {
        result = pContext->hog_detector.Detect(
            pContext->sonar_holder.cart_image(),
            pContext->sonar_holder.cart_image_mask(),
            annotations,
            locations,
            weights);
    }

    cv::Mat output_image;
    pContext->sonar_holder.cart_image().copyTo(output_image);

    cv::Mat annotation_image;
    pContext->sonar_holder.cart_image().copyTo(annotation_image);

    image_util::draw_contour(annotation_image, annotation_image, cv::Scalar(0, 0, 255), annotations);
    cv::imshow("annotation_image", annotation_image);

    double target_orientation = DBL_EPSILON;

    if (result) {

        if (locations.size() > 1 && pContext->dataset_info.detection_settings().enable_location_best_weight_filter) {

            size_t index = sonarlog_target_tracking::common::find_best_weight_location_index(weights);

            double weight = weights[index];
            cv::RotatedRect loc = locations[index];

            weights.clear();
            locations.clear();

            weights.push_back(weight);
            locations.push_back(loc);
        }

        target_orientation = locations[0].angle;


        if (pContext->dataset_info.detection_settings().show_classifier_weights) {
            image_util::draw_locations(output_image, locations, weights, output_image);
        }
        else {
            image_util::draw_locations(output_image, locations, output_image);
        }

        pContext->detected_sample_count++;
    }
    else {
       pContext->failed_sample_count++;
    }

    DetectionEval detection_eval(locations, annotations, output_image.size());

    cv::imshow("overlap_region_image", detection_eval.overlap_region_image());

    pContext->sample_count++;

    base::Time elapsed_time = base::Time::now()-pContext->start_time;
    double fps = pContext->sample_count/elapsed_time.toSeconds();

    pContext->precision_sum += detection_eval.precision();
    pContext->accuracy_sum += detection_eval.accuracy();
    pContext->recall_sum += detection_eval.recall();
    pContext->fall_out_sum += detection_eval.fall_out();
    pContext->overlap_region_sum += detection_eval.overlap_region();
    pContext->f1_score_sum += detection_eval.f1_score();

    double accuracy_average = pContext->accuracy_sum / pContext->sample_count;
    double precision_average = pContext->precision_sum / pContext->sample_count;
    double recall_average = pContext->recall_sum / pContext->sample_count;
    double fall_out_average = pContext->fall_out_sum / pContext->sample_count;
    double overlap_region_average = pContext->overlap_region_sum / pContext->sample_count;
    double f1_score_average = pContext->f1_score_sum /pContext->sample_count;

    printf("Sample index: %d, Sample count: %d\n", sample_index, pContext->sample_count);
    printf("  Detected: %d, Failed: %d, Frame per seconds: %lf, Target Orientation %lf\n",
        pContext->detected_sample_count, pContext->failed_sample_count, fps, target_orientation);
    printf("  Accuracy: %lf, Precision %lf, Recall: %lf, Fall-out: %lf, Overlap Region: %lf, F1 Score: %lf\n",
        accuracy_average, precision_average, recall_average, fall_out_average, overlap_region_average, f1_score_average);

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
    context.hog_detector.set_show_descriptor(context.dataset_info.training_settings().hog_show_descriptor);
    context.hog_detector.set_detection_scale_factor(context.dataset_info.detection_settings().detection_scale_factor);
    context.hog_detector.set_window_stride(context.dataset_info.detection_settings().hog_detector_stride);
    context.hog_detector.set_image_scale(context.dataset_info.detection_settings().hog_detector_scale);
    context.hog_detector.set_orientation_step(context.dataset_info.detection_settings().find_target_orientation_step);

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
    context.precision_sum = 0;
    context.accuracy_sum = 0;
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
    std::string evaluation_filepath = context.dataset_info.training_settings().output_directory_path()+"/"+
        context.dataset_info.detection_settings().evaluation_filename;
    context.detection_eval_list.csv_write(evaluation_filepath);
    printf("Finished\n");

    return 0;
}
