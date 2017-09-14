#include <boost/filesystem.hpp>
#include <sonar_processing/ImageUtil.hpp>
#include "ArgumentParser.hpp"
#include "Common.hpp"
#include "DatasetInfo.hpp"
#include "DetectionEval.hpp"
#include "EvaluationSettings.hpp"

using namespace sonarlog_target_tracking;
using namespace sonar_processing;

#define EVAL_FOLDER_NAME "eval_output"

std::string get_training_settings_string(
    const PreprocessingSettings& preprocessing_settings,
    const TrainingSettings& training_settings)
{
    std::stringstream ss;
    ss << "bfe-" << preprocessing_settings.border_filter_enable << "-";
    ss << "bft-" << preprocessing_settings.border_filter_type << "-";
    ss << "mdfe-" << preprocessing_settings.mean_diff_filter_enable << "-";
    ss << "piv-" << training_settings.positive_input_validate << "-";
    ss << "htsf-" << training_settings.hog_training_scale_factor << "-";
    ss << "htws-" << training_settings.hog_window_size.width;
    ss << "x" << training_settings.hog_window_size.height;
    return ss.str();
}

std::string get_detection_settings_string(
    const DetectionSettings& detection_settings)
{
    std::stringstream ss;
    ss << "elbwf-" << detection_settings.enable_location_best_weight_filter << "-";
    ss << "dsf-" << detection_settings.detection_scale_factor << "-";
    ss << "ftoe-" << detection_settings.find_target_orientation_enable << "-";
    if (detection_settings.find_target_orientation_enable) {
        ss << "ftos-" << detection_settings.find_target_orientation_step << "-";
    }
    else {
        ss << "ftos-##-";
    }
    ss << "hdsc-" << detection_settings.hog_detector_scale << "-";
    ss << "hdst-" <<  detection_settings.hog_detector_stride.width << "x" << detection_settings.hog_detector_stride.height;
    return ss.str();
}

std::string get_settings_string(
    const PreprocessingSettings& preprocessing_settings,
    const TrainingSettings& training_settings,
    const DetectionSettings& detection_settings)
{
    std::stringstream ss;
    ss << get_training_settings_string(preprocessing_settings, training_settings) << "-";
    ss << get_detection_settings_string(detection_settings);
    return ss.str();
}

void perform_hog_detector_test(
    const PreprocessingSettings& preprocessing_settings,
    const TrainingSettings& training_settings,
    const DetectionSettings& detection_settings,
    const std::vector<base::samples::Sonar>& test_samples,
    const std::vector<std::vector<cv::Point> >& test_annotations,
    const std::string& model_filename,
    EvaluationHolder& evaluation_holder,
    bool show_detection_result = false)
{
    sonar_processing::SonarHolder sonar_holder;
    sonar_processing::HOGDetector hog_detector;

    hog_detector.set_windown_size(training_settings.hog_window_size);
    hog_detector.set_show_descriptor(training_settings.hog_show_descriptor);
    hog_detector.set_detection_scale_factor(detection_settings.detection_scale_factor);
    hog_detector.set_window_stride(detection_settings.hog_detector_stride);
    hog_detector.set_image_scale(detection_settings.hog_detector_scale);
    hog_detector.set_orientation_step(detection_settings.find_target_orientation_step);

    sonar_processing::SonarImagePreprocessing preprocessing;
    sonarlog_target_tracking::common::load_preprocessing_settings(preprocessing_settings, preprocessing);
    hog_detector.set_sonar_image_processing(preprocessing);
    hog_detector.set_sonar_image_size(preprocessing_settings.image_max_size);

    hog_detector.LoadSVMTrain(model_filename);

    double accuracy_sum = 0.0;
    double precision_sum = 0.0;
    double recall_sum = 0.0;
    double fall_out_sum = 0.0;
    double overlap_region_sum = 0.0;
    double f1_score_sum = 0.0;

    size_t sample_count = 0;
    evaluation_holder.detected_sample_count = 0;
    evaluation_holder.failed_sample_count = 0;

    base::Time start_time = base::Time::now();

    for (size_t i = 0; i < test_samples.size(); i++) {

        sonarlog_target_tracking::common::load_sonar_holder(test_samples[i], sonar_holder, preprocessing_settings.image_max_size);
        std::vector<cv::Point> annotations = test_annotations[i];

        if (preprocessing_settings.image_max_size != cv::Size(-1, -1)) {
            image_util::resize_points(
                annotations,
                annotations,
                sonar_holder.cart_width_factor(),
                sonar_holder.cart_height_factor());
        }

        std::vector<cv::RotatedRect> locations;
        std::vector<double> weights;

        bool result = false;
        if (detection_settings.find_target_orientation_enable) {
            result = hog_detector.Detect(
                sonar_holder.cart_image(),
                sonar_holder.cart_image_mask(),
                locations,
                weights);
        }
        else {
            result = hog_detector.Detect(
                sonar_holder.cart_image(),
                sonar_holder.cart_image_mask(),
                annotations,
                locations,
                weights);
        }

        if (result) {
            if (locations.size() > 1 &&
                detection_settings.enable_location_best_weight_filter) {

                size_t index = sonarlog_target_tracking::common::find_best_weight_location_index(weights);

                double weight = weights[index];
                cv::RotatedRect loc = locations[index];

                weights.clear();
                locations.clear();

                weights.push_back(weight);
                locations.push_back(loc);

            }
            evaluation_holder.detected_sample_count++;
        }
        else {
            evaluation_holder.failed_sample_count++;
        }

        DetectionEval detection_eval(locations, annotations, sonar_holder.cart_image().size());

        sample_count++;

        base::Time elapsed_time = base::Time::now()-start_time;
        double fps = sample_count/elapsed_time.toSeconds();
        precision_sum += detection_eval.precision();
        accuracy_sum += detection_eval.accuracy();
        recall_sum += detection_eval.recall();
        fall_out_sum += detection_eval.fall_out();
        overlap_region_sum += detection_eval.overlap_region();
        f1_score_sum += detection_eval.f1_score();

        evaluation_holder.frame_per_seconds_average = sample_count / elapsed_time.toSeconds();
        evaluation_holder.accuracy_average = accuracy_sum / sample_count;
        evaluation_holder.precision_average = precision_sum / sample_count;
        evaluation_holder.recall_average = recall_sum / sample_count;
        evaluation_holder.fall_out_average = fall_out_sum / sample_count;
        evaluation_holder.overlap_region_average = overlap_region_sum / sample_count;
        evaluation_holder.f1_score_average = f1_score_sum / sample_count;


        printf("Current Settings:\n");
        printf("  border-filter-enable [BFE]: %d\n", preprocessing_settings.border_filter_enable);
        printf("  border-filter-type [BFT]: %s\n", preprocessing_settings.border_filter_type.c_str());
        printf("  mean-diff-filter-enable [MDFE]: %d\n", preprocessing_settings.mean_diff_filter_enable);
        printf("  positive-input-validate [PIV]: %d\n", training_settings.positive_input_validate);
        printf("  hog-training-scale-factor [HTSF]: %lf\n", training_settings.hog_training_scale_factor);
        printf("  hog-training-window-size [HTWS]: %dx%d\n",
            training_settings.hog_window_size.width,
            training_settings.hog_window_size.height);
        printf("  enable-location-best-weight-filter [ELBWF]: %d\n", detection_settings.enable_location_best_weight_filter);
        printf("  detection-scale-factor [DSF]: %lf\n", detection_settings.detection_scale_factor);
        printf("  find-target-orientation-enable [FTOE]: %d\n", detection_settings.find_target_orientation_enable);
        if (detection_settings.find_target_orientation_enable)
            printf("  find-target-orientation-step [FTOS]: %lf\n", detection_settings.find_target_orientation_step);
        else
            printf("  find-target-orientation-step [FTOS]: -\n");
        printf("  hog-detector-scale [HDSc]: %lf\n", detection_settings.hog_detector_scale);
        printf("  hog-detector-stride [HDSt]: %dx%d\n",
            detection_settings.hog_detector_stride.width,
            detection_settings.hog_detector_stride.height);
        printf("Sample count: %d\n", sample_count);
        printf("  Detected: %d, Failed: %d, Frame per seconds: %lf\n",
            evaluation_holder.detected_sample_count,
            evaluation_holder.failed_sample_count,
            evaluation_holder.frame_per_seconds_average);
        printf("  Accuracy: %lf, Precision %lf, Recall: %lf, Fall-out: %lf, Overlap Region: %lf, F1 Score: %lf\n",
            evaluation_holder.accuracy_average,
            evaluation_holder.precision_average,
            evaluation_holder.recall_average,
            evaluation_holder.fall_out_average,
            evaluation_holder.overlap_region_average,
            evaluation_holder.f1_score_average);
        fflush(stdout);


        if (show_detection_result) {
            cv::Mat output_image;
            sonar_holder.cart_image().copyTo(output_image);

            if (detection_settings.show_classifier_weights) {
                image_util::draw_locations(output_image, locations, weights, output_image);
            }
            else {
                image_util::draw_locations(output_image, locations, output_image);
            }

            cv::imshow("overlap_region_image", detection_eval.overlap_region_image());
            cv::imshow("output", output_image);
            cv::waitKey(15);
        }
    }
}

void hog_detector_test(
    const PreprocessingSettings& preprocessing_settings,
    const TrainingSettings& training_settings,
    const DetectionSettings& detection_settings,
    const std::vector<base::samples::Sonar>& test_samples,
    const std::vector<std::vector<cv::Point> >& test_annotations,
    const std::string& model_filename,
    const std::string& result_filename,
    std::map<std::string, EvaluationHolder>& evaluation_holder_map,
    bool show_detection_result = false)
{
    EvaluationHolder evaluation_holder;
    memset(&evaluation_holder, 0, sizeof(EvaluationHolder));

    std::string settings_title = get_settings_string(preprocessing_settings, training_settings, detection_settings);

    if (evaluation_holder_map.find(settings_title) == evaluation_holder_map.end()) {
        perform_hog_detector_test(
            preprocessing_settings,
            training_settings,
            detection_settings,
            test_samples,
            test_annotations,
            model_filename,
            evaluation_holder,
            show_detection_result);

        std::cout << "Insert Evaluation for settings: " << settings_title << std::endl;
        evaluation_holder_map.insert(std::make_pair<std::string, EvaluationHolder>(settings_title, evaluation_holder));
        EvaluationPersistent persistent;
        persistent.save(result_filename, evaluation_holder_map);
    }
    else {
        std::cout << "It already have a evaluation for settings: " << std::endl;
        std::cout << "  " << settings_title << std::endl;
                printf("  Detected: %d, Failed: %d, Frame per seconds: %lf\n",
                    evaluation_holder_map[settings_title].detected_sample_count,
                    evaluation_holder_map[settings_title].failed_sample_count,
                    evaluation_holder_map[settings_title].frame_per_seconds_average);
                printf("  Accuracy: %lf, Precision %lf, Recall: %lf, Fall-out: %lf, Overlap Region: %lf, F1 Score: %lf\n",
                    evaluation_holder_map[settings_title].accuracy_average,
                    evaluation_holder_map[settings_title].precision_average,
                    evaluation_holder_map[settings_title].recall_average,
                    evaluation_holder_map[settings_title].fall_out_average,
                    evaluation_holder_map[settings_title].overlap_region_average,
                    evaluation_holder_map[settings_title].f1_score_average);
    }
}


void perform_hog_detector_training(
    const PreprocessingSettings& preprocessing_settings,
    const TrainingSettings& training_settings,
    const std::vector<base::samples::Sonar>& train_samples,
    const std::vector<std::vector<cv::Point> >& train_annotations,
    const std::string& model_filename)
{
    sonar_processing::HOGDetector hog_detector;
    sonar_processing::SonarImagePreprocessing preprocessing;

    sonarlog_target_tracking::common::load_training_settings(training_settings, hog_detector);
    sonarlog_target_tracking::common::load_preprocessing_settings(preprocessing_settings, preprocessing);

    hog_detector.set_sonar_image_processing(preprocessing);
    hog_detector.set_sonar_image_size(preprocessing_settings.image_max_size);
    hog_detector.set_show_positive_window(false);

    std::cout << "HOG Detector training..." << std::endl;
    hog_detector.Train(train_samples, train_annotations, model_filename);
    std::cout << "HOG Detector training finished" << std::endl;
}

void hog_detector_evaluation(
    const TrainingEvaluationSettings& tes,
    const std::vector<DetectionEvaluationSettings>& des,
    const DatasetInfo& dataset_info,
    const std::vector<base::samples::Sonar>& train_samples,
    const std::vector<std::vector<cv::Point> >& train_annotations,
    const std::vector<base::samples::Sonar>& test_samples,
    const std::vector<std::vector<cv::Point> >& test_annotations,
    bool show_detection_result = false)
{
    std::map<std::string, EvaluationHolder> evaluation_holder_map;

    PreprocessingSettings preprocessing_settings = dataset_info.preprocessing_settings();
    preprocessing_settings.border_filter_enable = tes.border_filter_enable;
    preprocessing_settings.border_filter_type = tes.border_filter_type;
    preprocessing_settings.mean_diff_filter_enable = tes.mean_diff_filter_enable;

    TrainingSettings training_settings = dataset_info.training_settings();
    training_settings.positive_input_validate = tes.positive_input_validate;
    training_settings.hog_training_scale_factor = tes.training_scale_factor_begin;

    std::string folder = training_settings.output_directory_path()+"/"+EVAL_FOLDER_NAME;
    std::string result_filename = folder+"/evaluation_results.csv";

    EvaluationPersistent persistent;
    persistent.open(result_filename, evaluation_holder_map);

    std::cout << "evaluation_holder_map_count: " << evaluation_holder_map.size() << std::endl;

    std::map<std::string, EvaluationHolder>::iterator it = evaluation_holder_map.begin();

    while (it != evaluation_holder_map.end()) {
        std::cout << it->first << ",";
        std::cout << it->second.frame_per_seconds_average << ",";
        std::cout << it->second.accuracy_average << ",";
        std::cout << it->second.precision_average << ",";
        std::cout << it->second.recall_average << ",";
        std::cout << it->second.fall_out_average << ",";
        std::cout << it->second.overlap_region_average << ",";
        std::cout << it->second.f1_score_average << ",";
        std::cout << it->second.detected_sample_count << ",";
        std::cout << it->second.failed_sample_count << "\n";
        it++;
    }

    while (training_settings.hog_training_scale_factor < (tes.training_scale_factor_end+tes.training_scale_factor_step)) {
        for (size_t i = 0; i < tes.windows.size(); i++) {

            training_settings.hog_window_size = tes.windows[i];

            std::string filename = get_training_settings_string(preprocessing_settings, training_settings)+".yml";
            std::string model_filename = folder+"/"+filename;

            std::cout << "HOG Detector model filename" << model_filename << std::endl;
            if (!sonarlog_target_tracking::common::file_exists(model_filename)) {
                std::cout << "Perform Training" << std::endl;
                perform_hog_detector_training(
                    preprocessing_settings,
                    training_settings,
                    train_samples,
                    train_annotations,
                    model_filename);
            }

            for (size_t i = 0; i < des.size(); i++) {

                std::cout <<"============================================================\n";
                std::cout <<"Detection Settings: " << i << "\n";
                std::cout <<"============================================================\n";
                std::cout << des[i].to_string();
                std::cout << "SVM Model Training: " << model_filename << std::endl;

                DetectionSettings detection_settings = dataset_info.detection_settings();
                detection_settings.enable_location_best_weight_filter = des[i].enable_location_best_weight_filter;
                detection_settings.find_target_orientation_enable = des[i].find_target_orientation_enable;
                detection_settings.detection_scale_factor = des[i].detection_scale_factor_begin;
                while (detection_settings.detection_scale_factor < (des[i].detection_scale_factor_end+des[i].detection_scale_factor_step)) {

                    detection_settings.hog_detector_scale = des[i].hog_detector_scale_begin;
                    while (detection_settings.hog_detector_scale < (des[i].hog_detector_scale_end+des[i].hog_detector_scale_step)) {

                        std::vector<int>::const_iterator win_stride_it = des[i].hog_detector_strides.begin();

                        while (win_stride_it != des[i].hog_detector_strides.end()) {
                            int stride = *(win_stride_it++);
                            detection_settings.hog_detector_stride = cv::Size(stride, stride);

                            if (detection_settings.find_target_orientation_enable) {
                                std::vector<int>::const_iterator orientation_step_it = des[i].find_target_orientation_steps.begin();

                                while (orientation_step_it != des[i].find_target_orientation_steps.end()) {
                                    detection_settings.find_target_orientation_step = *(orientation_step_it++);
                                    hog_detector_test(
                                        preprocessing_settings,
                                        training_settings,
                                        detection_settings,
                                        test_samples,
                                        test_annotations,
                                        model_filename,
                                        result_filename,
                                        evaluation_holder_map,
                                        show_detection_result);
                                }
                            }
                            else {
                                hog_detector_test(
                                    preprocessing_settings,
                                    training_settings,
                                    detection_settings,
                                    test_samples,
                                    test_annotations,
                                    model_filename,
                                    result_filename,
                                    evaluation_holder_map,
                                    show_detection_result);
                            }
                        }

                        detection_settings.hog_detector_scale += des[i].hog_detector_scale_step;
                    }
                    detection_settings.detection_scale_factor += des[i].detection_scale_factor_step;
                }
            }

        }
        training_settings.hog_training_scale_factor +=tes.training_scale_factor_step;
    }
}

int main(int argc, char **argv) {

    ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    EvaluationSettings evaluation_settings(argument_parser.evaluation_settings_filename());
    DatasetInfo dataset_info(argument_parser.dataset_info_filename());

    std::string evaluation_directory = dataset_info.training_settings().output_directory_path()+"/"+EVAL_FOLDER_NAME;
    if (!sonarlog_target_tracking::common::file_exists(evaluation_directory)) {
        boost::filesystem::path dir(evaluation_directory);
        if (boost::filesystem::create_directory(dir)) {
            std::cout << "Cannot create the directory." << std::endl;
        }
    }

    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = dataset_info.entries();

    std::vector<TrainingEvaluationSettings> tes;
    tes.insert(
        tes.end(),
        evaluation_settings.training_evaluation_settings().begin(),
        evaluation_settings.training_evaluation_settings().end());

    std::vector<DetectionEvaluationSettings> des;
    des.insert(
        des.end(),
        evaluation_settings.detection_evaluation_settings().begin(),
        evaluation_settings.detection_evaluation_settings().end());

    std::vector<base::samples::Sonar> train_samples;
    std::vector<std::vector<cv::Point> > train_annotations;

    std::vector<std::vector<cv::Point> > test_annotations;
    std::vector<base::samples::Sonar> test_samples;

    for (size_t i=0; i<entries.size(); i++) {
        sonarlog_target_tracking::common::load_samples_from_dataset_entry(entries[i], test_samples, test_annotations);
        sonarlog_target_tracking::common::load_training_data_from_dataset_entry(entries[i], train_samples, train_annotations);
    }

    std::cout << "Total train samples: " << train_samples.size() << std::endl;
    std::cout << "Total train annotations: " << train_annotations.size() << std::endl;

    std::cout << "Total test samples: " << test_samples.size() << std::endl;
    std::cout << "Total test annotations: " << test_annotations.size() << std::endl;

    for (size_t i = 0; i < tes.size(); i++) {
        std::cout <<"============================================================\n";
        std::cout <<" Training Settings: " << i << "\n";
        std::cout <<"============================================================\n";
        std::cout << tes[i].to_string();
        hog_detector_evaluation(
            tes[i],
            des,
            dataset_info,
            train_samples,
            train_annotations,
            test_samples,
            test_annotations,
            argument_parser.show_detection_result());
    }

    return 0;
}
