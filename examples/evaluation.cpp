#include <stdint.h> 
#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "base/test_config.h"
#include "sonar_processing/ImageUtil.hpp"
#include "sonarlog_target_tracking/DetectionStats.hpp"

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

void detection_evaluate_from_file(const std::string& filename) {
    sonarlog_target_tracking::DetectionStats stats(filename);

    size_t total_frames = stats.frame_sizes().size();

    for (size_t k = 0; k < total_frames; k++) {
        cv::Size frame_size = stats.frame_sizes()[k];
        std::vector<cv::Point> ground_truth_points = stats.annotations()[k];

        cv::Mat ground_truth_mask = cv::Mat::zeros(frame_size, CV_8UC1);
        image_util::create_min_area_rect_mask(ground_truth_points, ground_truth_mask);
        cv::Mat ground_truth_mask_inv = 255-ground_truth_mask;

        std::vector<cv::RotatedRect> detection_results = stats.detection_results()[k];
        
        cv::Mat1b detection_mask = cv::Mat1b::zeros(frame_size);
        for (size_t l = 0; l < detection_results.size(); l++) {
            cv::Mat1b mask = cv::Mat1b::zeros(frame_size);
            image_util::create_rotated_rect_mask(detection_results[l], mask);
            detection_mask+=mask;
        }

        cv::Mat detection_mask_inv = 255-detection_mask;

        cv::Mat tp_mask, tn_mask, fp_mask, fn_mask;
        cv::bitwise_and(ground_truth_mask, detection_mask, tp_mask);
        cv::bitwise_and(ground_truth_mask_inv, detection_mask, fp_mask);
        cv::bitwise_and(ground_truth_mask, detection_mask_inv, fn_mask);
        cv::bitwise_and(ground_truth_mask_inv, detection_mask_inv, tn_mask);

        cv::Mat rgb[] = {
            fn_mask,
            tp_mask,
            fp_mask
        };

        cv::Mat3b canvas;
        cv::merge(rgb, 3, canvas);
        image_util::show_image("result", canvas, 2);
        cv::waitKey(15);

    }
    
}

int main(int argc, char **argv) {
    const std::string results_filename[] = {
        DATA_PATH_STRING + "/results/20161206-1642_001196-002416_gemini.0_detection_result.yml"
    };

    uint32_t sz = sizeof(results_filename)/sizeof(std::string);

    for (uint32_t k = 0; k < sz; k++) {
        std::cout << "Result filename: " << results_filename[k] << std::endl;
        detection_evaluate_from_file(results_filename[k]);
    }

    return 0;
}
