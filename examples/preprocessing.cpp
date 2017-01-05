#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <base/samples/Sonar.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/ml/ml.hpp>
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/SonarHolder.hpp"
#include "sonar_processing/SonarImagePreprocessing.hpp"
#include "sonar_processing/Utils.hpp"
#include "sonarlog_target_tracking/Common.hpp"
#include "sonarlog_annotation/AnnotationFileReader.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"

// #define PARAMS_EVALUATE

using namespace sonar_processing;
using namespace sonarlog_annotation;

static boost::filesystem::path g_log_file_path;

double perform_sonar_image_preprocessing(const SonarImagePreprocessing& sonar_image_processing, const std::vector<base::samples::Sonar>& samples, std::vector<std::vector<cv::Point> >& annotations, int start_sample=0, int sample_count=-1) {
    SonarHolder sonar_holder;
    
    double average_intersection_percentage=0;

    sample_count = (sample_count==-1) ? samples.size() : sample_count;

    size_t final_sample_index = start_sample+sample_count;

    int cnt = 0;
    for (size_t sample_index = start_sample; sample_index<samples.size() && sample_index<final_sample_index; sample_index++, cnt++) {

        // load sonar holder data structure
        sonarlog_target_tracking::common::load_sonar_holder(samples[sample_index], sonar_holder);

        // adjust annotation to fit the preprocessed image
        std::vector<cv::Point> annotation_points = annotations[sample_index];
        cv::Mat annotation_mask;
        sonarlog_target_tracking::common::adjust_annotation(sonar_holder.cart_size(), annotation_points, annotation_points, annotation_mask);

        // process the cart image
        cv::Mat preprocessed_image, preprocessed_mask;
        sonar_image_processing.Apply(sonar_holder, preprocessed_image, preprocessed_mask, 0.5);

#ifndef PARAMS_EVALUATE
        image_util::show_image("preprocessed_image", preprocessed_image, 2);
        image_util::show_image("preprocessed_mask", preprocessed_mask, 2);
        cv::waitKey(10);
#endif
        cv::Mat intersection_mask;
        preprocessed_mask.convertTo(preprocessed_mask, CV_8U, 255.0);
        cv::bitwise_and(annotation_mask, preprocessed_mask, intersection_mask);

        double annotation_area = cv::sum(annotation_mask)[0]/255.0;
        double preprocessed_area = cv::sum(preprocessed_mask)[0]/255.0;
        double intersection_area = cv::sum(intersection_mask)[0]/255.0;
        double intersection_percentage = intersection_area/(annotation_area+preprocessed_area-intersection_area);
        average_intersection_percentage+=intersection_percentage;

        printf("\033[u");
        printf("Sample: %ld - Intersection percentage: %f", sample_index+1, intersection_percentage);
        fflush(stdout);
    }
    printf("\n");
    return average_intersection_percentage/cnt;
}

void preprocessing_parameters_evaluate(const std::vector<base::samples::Sonar>& samples, std::vector<std::vector<cv::Point> >& annotations) {
    const float final_clip_limits[]={2.0, 3.0, 4.0};
    const int mean_filter_ksizes[]={5, 7, 9};
    const int mean_difference_filter_ksizes[]={25, 50, 75, 100};
    const int median_blur_filter_ksizes[]={5, 7};
    const int saliency_map_block_count[]={8, 10, 12};
    const float saliency_map_thresh_factors[]={0.2, 0.3, 0.4, 0.5, 0.6};

    int total_final_clip_limits = sizeof(final_clip_limits) / sizeof(float);
    int total_mean_filter_ksizes = sizeof(mean_filter_ksizes) / sizeof(int);
    int total_mean_difference_filter_ksizes = sizeof(mean_difference_filter_ksizes) / sizeof(int);
    int total_median_blur_filter_ksizes= sizeof(median_blur_filter_ksizes) / sizeof(int);
    int total_saliency_map_block_count= sizeof(saliency_map_block_count) / sizeof(int);
    int saliency_map_thresh_factors_count= sizeof(saliency_map_thresh_factors) / sizeof(float);

    bool enable_final_clip_limit=true;
    bool enable_mean_filter_ksize=true;
    bool enable_mean_difference_filter_ksize=true;
    bool enable_median_blur_filter_ksize=true;
    bool enable_saliency_map_block_count=true;
    bool enable_saliency_map_thresh_factor=true;

    SonarImagePreprocessing sonar_image_processing;
    
    std::ofstream file;
    file.open("evaluate.txt");
    for (int final_clip_limit_index=0; final_clip_limit_index<total_final_clip_limits; final_clip_limit_index++) {
        for (int mean_filter_ksize_index=0; mean_filter_ksize_index<total_mean_filter_ksizes; mean_filter_ksize_index++) {
            for (int mean_difference_filter_ksize_index=0; mean_difference_filter_ksize_index<total_mean_difference_filter_ksizes; mean_difference_filter_ksize_index++) {
                for (int median_blur_filter_ksize_index=0; median_blur_filter_ksize_index<total_median_blur_filter_ksizes; median_blur_filter_ksize_index++) {
                    for (int saliency_map_block_count_index=0; saliency_map_block_count_index<total_saliency_map_block_count; saliency_map_block_count_index++) {
                        for (int saliency_map_thresh_factor_index=0; saliency_map_thresh_factor_index<saliency_map_thresh_factors_count; saliency_map_thresh_factor_index++) {

                            printf("Sonar Image Preprocessing parameters\n");
                            printf(" - clahe_final_clip_limit: %f\n", final_clip_limits[final_clip_limit_index]);
                            printf(" - mean_filter_ksize: %d\n", mean_filter_ksizes[mean_filter_ksize_index]);
                            printf(" - mean_difference_filter_ksize: %d\n", mean_difference_filter_ksizes[mean_difference_filter_ksize_index]);
                            printf(" - median_blur_filter_ksize: %d\n", median_blur_filter_ksizes[median_blur_filter_ksize_index]);
                            printf(" - saliency_map_block_count: %d\n", saliency_map_block_count[saliency_map_block_count_index]);
                            printf(" - saliency_map_thresh_factor: %f\n", saliency_map_thresh_factors[saliency_map_thresh_factor_index]);
                            printf("\033[s");

                            sonar_image_processing.set_clahe_final_clip_limit(final_clip_limits[final_clip_limit_index]);
                            sonar_image_processing.set_mean_filter_ksize(mean_filter_ksizes[mean_filter_ksize_index]);
                            sonar_image_processing.set_mean_difference_filter_ksize(mean_difference_filter_ksizes[mean_difference_filter_ksize_index]);
                            sonar_image_processing.set_median_blur_filter_ksize(median_blur_filter_ksizes[median_blur_filter_ksize_index]);
                            sonar_image_processing.set_saliency_map_block_count(saliency_map_block_count[saliency_map_block_count_index]);
                            sonar_image_processing.set_saliency_map_thresh_factor(saliency_map_thresh_factors[saliency_map_thresh_factor_index]);
                            
                            int percentage_cnt = 0;
                            double percentage = 0;

                            for (int i = 0; i <= 1200; i+=50) {
                                percentage += perform_sonar_image_preprocessing(sonar_image_processing, samples, annotations, i, 5);
                                percentage_cnt++;    
                            }

                            percentage = percentage / (double)percentage_cnt;

                            printf("Average intersection percentage: %f\n", percentage);
                            file << "clahe_final_clip_limit[" << final_clip_limits[final_clip_limit_index] << "]|";
                            file << "mean_filter_ksize[" << mean_filter_ksizes[mean_filter_ksize_index] << "]|";
                            file << "mean_difference_filter_ksize[" << mean_difference_filter_ksizes[mean_difference_filter_ksize_index] << "]|";
                            file << "median_blur_filter_ksize[" << median_blur_filter_ksizes[median_blur_filter_ksize_index] << "]|";
                            file << "saliency_map_block_count[" << saliency_map_block_count[saliency_map_block_count_index] << "]|";
                            file << "saliency_map_thresh_factor[" << saliency_map_thresh_factors[saliency_map_thresh_factor_index] << "]|";
                            file << "\t" << percentage;
                            file << "\n";

                            if (!enable_saliency_map_thresh_factor) {
                                break;
                            }
                        }

                        if (!enable_saliency_map_block_count) {
                            break;
                        }
                    }
                    
                    if (!enable_median_blur_filter_ksize) {
                        break;
                    }

                }
                
                if (!enable_mean_difference_filter_ksize) {
                    break;
                }
            }
            
            if (!enable_mean_filter_ksize) {
                break;
            }
        }

        if (!enable_final_clip_limit) {
            break;
        }
    }
    file.close();
}

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        "/home/gustavoneves/masters_degree/dataset/logs/20161206-1642_001196-002416_gemini.0.log"
    };

    const std::string logannotation_files[] = {
        "/home/gustavoneves/masters_degree/dataset/logs/20161206-1642_001196-002416_gemini.0_annotation.yml"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    for (uint32_t i = 0; i < sz; i++) {
        boost::filesystem::path logpath(logfiles[i]);
        std::cout << "parent_path: " << logpath.parent_path() << std::endl;
        std::cout << "stem: " << logpath.stem() << std::endl;

        std::string parent_path = logpath.parent_path().string();
        std::string log_name = logpath.stem().string();

        g_log_file_path = boost::filesystem::path(parent_path+"/"+log_name);

        if (!boost::filesystem::exists(g_log_file_path)){
            boost::filesystem::create_directory(g_log_file_path);
        }

        // load log samples
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        // load sonar samples
        std::vector<base::samples::Sonar> samples;
        sonarlog_target_tracking::common::load_samples(stream, samples);

        // load log annotations
        std::vector<std::vector<cv::Point> > annotations;
        sonarlog_target_tracking::common::load_log_annotation(logannotation_files[i], "jequitaia", annotations);

#ifdef PARAMS_EVALUATE
        // perform the parameters evaluation
        preprocessing_parameters_evaluate(samples, annotations);
#else
        // test preprocessing parameters evaluation
        SonarImagePreprocessing sonar_image_processing;
        sonar_image_processing.set_clahe_final_clip_limit(2.0);
        sonar_image_processing.set_mean_filter_ksize(5);
        sonar_image_processing.set_mean_difference_filter_ksize(75);
        sonar_image_processing.set_median_blur_filter_ksize(5);
        sonar_image_processing.set_saliency_map_block_count(12);
        sonar_image_processing.set_saliency_map_thresh_factor(0.3);
        sonar_image_processing.set_saliency_map_scale_factor(0.25);
        for (int i = 0; i <= samples.size(); i+=50) {
            perform_sonar_image_preprocessing(sonar_image_processing, samples, annotations, i, 5);
        }
#endif

    }
}
