#include <cstdio>
#include "sonar_processing/ImageUtil.hpp"
#include "DetectionEval.hpp"

namespace sonarlog_target_tracking
{

DetectionEval::DetectionEval(
    const std::vector<cv::RotatedRect>& locations,
    const std::vector<cv::Point>& annotations,
    cv::Size size)
    : locations_(locations)
    , annotations_(annotations)
    , frame_size_(size)
{
    EvalCalculate();
}

DetectionEval::~DetectionEval()
{
}

void DetectionEval::EvalCalculate()
{

    // ground truth mask
    cv::Mat1b ground_truth_mask = cv::Mat1b::zeros(frame_size_);
    sonar_processing::image_util::create_min_area_rect_mask(annotations_, ground_truth_mask);

    // inverted ground truth mask
    cv::Mat ground_truth_mask_inv = 255-ground_truth_mask;

    // create detection result mask
    cv::Mat1b detection_mask = cv::Mat1b::zeros(frame_size_);
    sonar_processing::image_util::create_rotated_rect_mask(locations_, detection_mask);

    // inverted detection result mask
    cv::Mat detection_mask_inv = 255-detection_mask;

    CalculateIntersectionArea(
        ground_truth_mask,
        ground_truth_mask_inv,
        detection_mask,
        detection_mask_inv);
}

void DetectionEval::CalculateIntersectionArea(
    const cv::Mat& ground_truth_mask,
    const cv::Mat& ground_truth_mask_inv,
    const cv::Mat& detection_mask,
    const cv::Mat& detection_mask_inv)
{
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

    cv::merge(rgb, 3, intersection_area_image_);

    true_positive_ = cv::sum(tp_mask)[0] / 255;
    false_positive_ = cv::sum(fp_mask)[0] / 255;
    true_negative_ = cv::sum(tn_mask)[0] / 255;
    false_negative_ = cv::sum(fn_mask)[0] / 255;

    true_positive_rate_ = true_positive_ / (true_positive_ + false_negative_);
    false_positive_rate_ = false_positive_ / (false_positive_ + true_negative_);

    accuracy_ = (true_positive_) / (true_positive_ + false_positive_ + false_negative_);
}

} // namespace sonarlog_target_tracking
