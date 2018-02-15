#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rock_util/LogReader.hpp>
#include <sonar_processing/HOGDetector.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/Utils.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DetectionEval.hpp"
#include "DetectionResult.hpp"
#include "DatasetInfo.hpp"

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

struct Context {
    sonar_processing::HOGDetector hog_detector;
    sonar_processing::SonarHolder sonar_holder;

    std::vector<std::vector<cv::Point> > annotations;
    std::string name;

    base::Time start_time;

    size_t sample_count;
    size_t annotated_sample_count;
    size_t detected_sample_count;

    size_t true_positive_count;
    size_t false_positive_count;
    size_t true_negative_count;
    size_t false_negative_count;

    DetectionEvalList detection_eval_list;
    DatasetInfo dataset_info;
    DetectionResult detection_result;

    bool is_positive_sample;

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

    cv::Size sonar_image_size = pContext->dataset_info.preprocessing_settings().image_max_size;

    pContext->sonar_holder.Reset(
        sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count,
        sonar_image_size
    );

    bool has_annotation = !(pContext->annotations.empty() || pContext->annotations[sample_index].empty());

    if (!has_annotation && pContext->is_positive_sample &&
        !pContext->dataset_info.detection_settings().include_no_annotated_samples) {
        printf("There is no annotation for this sample.\n");
        return;
    }


    std::vector<cv::Point> annotations;

    if (has_annotation) {
        annotations = pContext->annotations[sample_index];

        if (sonar_image_size != cv::Size(-1, -1)) {
            image_util::resize_points(
                annotations,
                annotations,
                pContext->sonar_holder.cart_width_factor(),
                pContext->sonar_holder.cart_height_factor());
        }
    }

    std::vector<cv::RotatedRect> locations;
    std::vector<double> weights;
    bool result;

    double image_scale = (pContext->is_positive_sample)
                            ? pContext->dataset_info.detection_settings().hog_detector_positive_scale
                            : pContext->dataset_info.detection_settings().hog_detector_negative_scale;

    pContext->hog_detector.set_image_scale(image_scale);

    if (!has_annotation ||
        pContext->dataset_info.detection_settings().find_target_orientation_enable) {



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

    double target_orientation = DBL_EPSILON;

    DetectionResultItem detection_result_item;
    detection_result_item.type = (pContext->is_positive_sample) ? 1 : -1;
    detection_result_item.name = pContext->name;
    detection_result_item.result = (result) ? 1 : 0;
    detection_result_item.weight = 0.0;
    detection_result_item.overlap = 0.0;

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

        detection_result_item.weight = weights[0];
        target_orientation = locations[0].angle;

        if (pContext->dataset_info.detection_settings().show_classifier_weights) {
            image_util::draw_locations(output_image, locations, weights, output_image);
        }
        else {
            image_util::draw_locations(output_image, locations, output_image);
        }

        pContext->detected_sample_count++;
    }

    pContext->sample_count++;

    base::Time elapsed_time = base::Time::now()-pContext->start_time;
    double fps = pContext->sample_count/elapsed_time.toSeconds();

    printf("[%s] Sample index: %d, Sample count: %d\n", (pContext->is_positive_sample) ? "Positive" : "Negative",
        sample_index, pContext->sample_count);
    printf("  Detected: %d, Frame per seconds: %lf, Target Orientation %lf\n", pContext->detected_sample_count, fps, target_orientation);


    if (has_annotation && pContext->is_positive_sample) {
        DetectionEval detection_eval(locations, annotations, output_image.size());
        pContext->annotated_sample_count++;
        pContext->precision_sum += detection_eval.precision();
        pContext->accuracy_sum += detection_eval.accuracy();
        pContext->recall_sum += detection_eval.recall();
        pContext->fall_out_sum += detection_eval.fall_out();
        pContext->overlap_region_sum += detection_eval.overlap_region();
        pContext->f1_score_sum += detection_eval.f1_score();

        double accuracy_average = pContext->accuracy_sum / pContext->annotated_sample_count;
        double precision_average = pContext->precision_sum / pContext->annotated_sample_count;
        double recall_average = pContext->recall_sum / pContext->annotated_sample_count;
        double fall_out_average = pContext->fall_out_sum / pContext->annotated_sample_count;
        double overlap_region_average = pContext->overlap_region_sum / pContext->annotated_sample_count;
        double f1_score_average = pContext->f1_score_sum / pContext->annotated_sample_count;

        detection_result_item.overlap = detection_eval.overlap_region();

        if (result) {
            if (detection_eval.overlap_region() >
                pContext->dataset_info.detection_settings().overlap_threshold)
                pContext->true_positive_count++;
            else
                pContext->false_positive_count++;
        }
        else {
            pContext->false_negative_count++;
        }

        printf("  Accuracy: %lf, Precision %lf, Recall: %lf, Fall-out: %lf, Overlap Region: %lf, F1 Score: %lf\n",
            detection_eval.accuracy(), detection_eval.precision(), detection_eval.recall(),
            detection_eval.fall_out(), detection_eval.overlap_region(), detection_eval.f1_score());

        pContext->detection_eval_list.add_detection_evaluation(detection_eval);

        cv::Mat annotation_image;
        pContext->sonar_holder.cart_image().copyTo(annotation_image);

        image_util::draw_contour(annotation_image, annotation_image, cv::Scalar(0, 0, 255), annotations);
        cv::imshow("annotation_image", annotation_image);
        cv::imshow("overlap_region_image", detection_eval.overlap_region_image());
    }
    else if (!has_annotation && pContext->is_positive_sample) {
        printf("  There is no evaluation data for this sample\n");
    }
    else if (!pContext->is_positive_sample) {

        cv::Size sz = output_image.size();
        annotations.push_back(cv::Point(0, 0));
        annotations.push_back(cv::Point(sz.width-1, 0));
        annotations.push_back(cv::Point(sz.width-1, sz.height-1));
        annotations.push_back(cv::Point(0, sz.height-1));

        DetectionEval detection_eval(locations, annotations, output_image.size());
        detection_result_item.overlap = detection_eval.overlap_region();

        printf("  Accuracy: %lf, Precision %lf, Recall: %lf, Fall-out: %lf, Overlap Region: %lf, F1 Score: %lf\n",
            detection_eval.accuracy(), detection_eval.precision(), detection_eval.recall(),
            detection_eval.fall_out(), detection_eval.overlap_region(), detection_eval.f1_score());

        cv::Mat annotation_image;
        pContext->sonar_holder.cart_image().copyTo(annotation_image);

        image_util::draw_contour(annotation_image, annotation_image, cv::Scalar(0, 0, 255), annotations);
        cv::imshow("annotation_image", annotation_image);
        cv::imshow("overlap_region_image", detection_eval.overlap_region_image());

        if (!result) {
            pContext->true_negative_count++;
        }
        else {
            pContext->false_positive_count++;
        }
    }


    if (has_annotation || !pContext->is_positive_sample) {
        printf("  True Positive Count: %d, False Positive Count: %d, True Negative Count: %d, False Negative Count: %d\n",
            pContext->true_positive_count, pContext->false_positive_count, pContext->true_negative_count, pContext->false_negative_count);
        pContext->detection_result.add(detection_result_item);
    }


    fflush(stdout);
    cv::imshow("output", output_image);
    cv::waitKey(15);
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
    context.hog_detector.set_detection_minimum_weight(context.dataset_info.detection_settings().detection_minimum_weight);
    context.hog_detector.set_window_stride(context.dataset_info.detection_settings().hog_detector_stride);
    context.hog_detector.set_image_scale(context.dataset_info.detection_settings().hog_detector_positive_scale);
    context.hog_detector.set_orientation_step(context.dataset_info.detection_settings().find_target_orientation_step);
    context.hog_detector.set_orientation_range(context.dataset_info.detection_settings().find_target_orientation_range);

    std::vector<sonarlog_target_tracking::DatasetInfoEntry> positive_entries = context.dataset_info.positive_entries();

    sonar_processing::SonarImagePreprocessing preprocessing;
    sonarlog_target_tracking::common::load_preprocessing_settings(context.dataset_info.preprocessing_settings(), preprocessing);
    context.hog_detector.set_sonar_image_processing(preprocessing);
    context.hog_detector.set_sonar_image_size(context.dataset_info.preprocessing_settings().image_max_size);

    std::cout << "SVM Model Training: " << context.dataset_info.training_settings().full_model_filename() << std::endl;
    context.hog_detector.LoadSVMTrain(context.dataset_info.training_settings().full_model_filename());

    context.sample_count = 0;
    context.annotated_sample_count = 0;
    context.detected_sample_count = 0;
    context.true_positive_count = 0;
    context.false_positive_count = 0;
    context.true_negative_count = 0;
    context.false_negative_count = 0;

    context.precision_sum = 0;
    context.accuracy_sum = 0;
    context.start_time = base::Time::now();

    context.is_positive_sample = true;
    context.detection_result.reset();

    printf("Processing Positive Samples\n");
    for (size_t i=0; i<positive_entries.size(); i++) {
        std::cout << "Annotation Filename: " << positive_entries[i].annotation_filename << std::endl;

        context.annotations.clear();
        if (!positive_entries[i].annotation_filename.empty() && common::file_exists(positive_entries[i].annotation_filename)) {
            common::load_log_annotation(
                positive_entries[i].annotation_filename,
                positive_entries[i].annotation_name,
                context.annotations);
        }

        printf("Processing log: %s\n", positive_entries[i].log_filename.c_str());
        context.name = positive_entries[i].name;
        context.hog_detector.reset_detection_stats();
        common::exec_samples_from_dataset_entry(positive_entries[i], sample_receiver_callback, &context);
        printf("\n");
    }

    std::vector<sonarlog_target_tracking::DatasetInfoEntry> negative_entries = context.dataset_info.negative_entries();

    context.is_positive_sample = false;

    printf("Processing Negative Samples\n");
    for (size_t i=0; i<negative_entries.size(); i++) {
        std::cout << "Annotation Filename: " << negative_entries[i].annotation_filename << std::endl;

        context.annotations.clear();
        printf("Processing log: %s\n", negative_entries[i].log_filename.c_str());
        context.hog_detector.reset_detection_stats();
        context.name = negative_entries[i].name;
        common::exec_samples_from_dataset_entry(negative_entries[i], sample_receiver_callback, &context);
        printf("\n");
    }

    printf("Save results into CSV file.\n");
    std::string evaluation_filepath = context.dataset_info.training_settings().output_directory_path()+"/"+
        context.dataset_info.detection_settings().evaluation_filename;
    context.detection_eval_list.csv_write(evaluation_filepath);

    std::string detection_result_filepath = context.dataset_info.training_settings().output_directory_path()+"/"+
        context.dataset_info.training_settings().model_filename+"-result.csv";

    context.detection_result.csv_write(detection_result_filepath);

    printf("True Positive Count: %d, False Positive Count: %d, True Negative Count: %d, False Negative Count: %d\n",
        context.true_positive_count, context.false_positive_count, context.true_negative_count, context.false_negative_count);

    printf("Finished\n");

    return 0;
}
