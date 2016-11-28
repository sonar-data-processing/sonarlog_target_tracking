#include "base/MathUtil.hpp"
#include "base/Plot.hpp"
#include "sonar_processing/BasicOperations.hpp"
#include "sonar_processing/Features.hpp"
#include "sonar_processing/FrequencyDomain.hpp"
#include "sonar_processing/ImageUtils.hpp"
#include "sonar_processing/ROI.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/Utils.hpp"
#include "cartesian_processing.hpp"

using namespace sonar_processing;

void CartesianProcessing::Process() {
    ShowCartesianLineLimits();
    BuildAnnotationsMask();
    // ShowAnnotationsMask();
    DetectRegionOfInterest();
    BackgroundEstimation();
    BuildCartesianMask(dst_sonar_holder_.cart_image_mask(), cart_image_mask_);
    FrequencyNoiseRemoval();
    GradientFilter();
    SaliencyMap();
    Thresholding();
    // ProcessGradientImage();
    // ShowGradientAndAnnotation();
}

void CartesianProcessing::ImageScale(const cv::Mat& in, cv::Mat& out) {
    cv::Size sz = cv::Size(in.size().width * scale_factor_,
                           in.size().height * scale_factor_);
    cv::resize(in, out, sz);
}

void CartesianProcessing::ShowScaledImage(const std::string& title, const cv::Mat& image) {
    cv::Mat scaled_image;
    ImageScale(image, scaled_image);
    cv::imshow(title, scaled_image);
}

void CartesianProcessing::ShowCartesianLineLimits() {
    cv::Mat cart_limits_image;
    DrawCartesianLineLimits(cart_limits_image);
    ShowScaledImage("cartesian_limits", cart_limits_image);
}

void CartesianProcessing::DrawCartesianLineLimits(cv::Mat& out_image){
    cv::cvtColor(src_sonar_holder_.cart_image(), out_image, CV_GRAY2BGR);
    for (size_t line = 0; line < src_sonar_holder_.cart_size().height; line++) {
        int x0, x1;
        src_sonar_holder_.cart_line_limits(line, x0, x1);
        if (x0 != -1 && x1 != -1) {
            cv::line(out_image, cv::Point(x0, line), cv::Point(x0+1, line), cv::Scalar(0, 0, 255), 2);
            cv::line(out_image, cv::Point(x1, line), cv::Point(x1-1, line), cv::Scalar(0, 0, 255), 2);
        }
    }
}

void CartesianProcessing::DetectRegionOfInterest() {
    sonar_processing::roi::cartesian::bins_of_interest(src_sonar_holder_, start_bin_, final_bin_);

    sonar_processing::basic_operations::line_indices_from_bin(src_sonar_holder_, start_bin_, start_line_indices_);
    sonar_processing::basic_operations::line_indices_from_bin(src_sonar_holder_, final_bin_, final_line_indices_);

    cv::Mat roi_image;
    cv::cvtColor(src_sonar_holder_.cart_image(), roi_image, CV_GRAY2BGR);

    std::vector<cv::Point2f> start_line_points;
    std::vector<cv::Point2f> final_line_points;
    src_sonar_holder_.cart_points(start_line_indices_, start_line_points);
    src_sonar_holder_.cart_points(final_line_indices_, final_line_points);

    image_utils::draw_line(roi_image, start_line_points.begin(), start_line_points.end(), cv::Scalar(0, 0, 255));
    image_utils::draw_line(roi_image, final_line_points.begin(), final_line_points.end(), cv::Scalar(0, 0, 255));

    src_sonar_holder_.CopyTo(dst_sonar_holder_, start_line_indices_, final_line_indices_);
}

void CartesianProcessing::BackgroundEstimation() {
    cv::Size ksize = cv::Size(300, 300);
    cv::Mat cart_image;
    src_sonar_holder_.cart_image().convertTo(cart_image, CV_8U, 255);
    cv::boxFilter(cart_image, cart_background_image_, CV_8U, ksize);
}

void CartesianProcessing::GradientFilter() {
    cv::Mat cart_image_8u;
    dst_sonar_holder_.cart_image().convertTo(cart_image_8u, CV_8U, 255.0);
    ApplyGradientFilter(cart_image_8u, cart_gradient_image_, cart_image_mask_);
    cart_gradient_image_ -= cart_background_image_;
    cv::normalize(cart_gradient_image_, cart_gradient_image_, 0, 255, cv::NORM_MINMAX, CV_8U, cart_image_mask_);
    ShowScaledImage("gradient_image", cart_gradient_image_);
}

void CartesianProcessing::ApplyGradientFilter(cv::InputArray src_arr, cv::OutputArray dst_arr, cv::InputArray mask) {
    cv::Mat src = src_arr.getMat();
    CV_Assert(src.depth() == CV_8U);
    cv::Mat mat;
    preprocessing::gradient_filter(src, mat);
    cv::normalize(mat, mat, 0, 255, cv::NORM_MINMAX, CV_8U, mask);
    mat.copyTo(dst_arr, mask);
}

void CartesianProcessing::FrequencyNoiseRemoval() {
    cv::Mat src = dst_sonar_holder_.raw_image();
    cv::Mat mask = cv::Mat(dst_sonar_holder_.bins_mask()).reshape(1, dst_sonar_holder_.beam_count());
    cv::Mat noise_removed, dst;
    frequency_domain::filters::noise_removal(src, noise_removed, 30, 2);
    noise_removed.copyTo(dst, mask);

    std::vector<float> bins;
    image_utils::mat2vector(dst, bins);
    dst_sonar_holder_.ResetBins(bins);
    // ShowScaledImage("noise_removal", dst_sonar_holder_.cart_image());
}

void CartesianProcessing::BuildAnnotationsMask() {
    std::map<std::string, std::vector<cv::Point2f> >::const_iterator it = annotations_.begin();
    while (it != annotations_.end()) {
        std::vector<cv::Point> contour;
        point2f_to_point(it->second, contour);
        cv::Mat mask;
        BuildContourMask(contour, mask);
        annotations_mask_.insert(std::make_pair(it->first, mask));
        it++;
    }
}

void CartesianProcessing::ShowAnnotationsMask() {
    std::map<std::string, cv::Mat>::iterator it = annotations_mask_.begin();
    while (it != annotations_mask_.end()) {
        ShowScaledImage(it->first, it->second);
        it++;
    }
}

void CartesianProcessing::BuildContourMask(const std::vector<cv::Point>& contour, cv::OutputArray dst_arr) {
    CV_Assert(!contour.empty());
    cv::Mat contour_image = cv::Mat::zeros(src_sonar_holder_.cart_image().size(), src_sonar_holder_.cart_image().type());
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours(contour_image, contours, -1, cv::Scalar(255), CV_FILLED);
    contour_image.copyTo(dst_arr);
}

void CartesianProcessing::DrawContour(const std::vector<cv::Point>& contour, cv::OutputArray dst_arr, cv::Scalar color) {
    CV_Assert(!contour.empty());
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours(dst_arr, contours, -1, color, 2);
}

void CartesianProcessing::ShowGradientAndAnnotation() {
    cv::Mat canvas;
    cv::cvtColor(cart_gradient_image_, canvas, CV_GRAY2BGR);
    std::map<std::string, std::vector<cv::Point2f> >::const_iterator it = annotations_.begin();

    while (it != annotations_.end()) {
        std::vector<cv::Point> contour;
        point2f_to_point(it->second, contour);
        DrawContour(contour, canvas, cv::Scalar(0, 0, 255));
        it++;
    }

    ShowScaledImage("gradient_and_annotation", canvas);
}

void CartesianProcessing::ProcessGradientImage() {
    static base::Plot mean_plot;
    static base::Plot stddev_plot;

    const int window_size = 50;
    const cv::Mat& cart_image_mask = dst_sonar_holder_.cart_image_mask();
    cv::Size cart_size = dst_sonar_holder_.cart_image().size();

    cv::Mat canvas;
    cv::cvtColor(cart_gradient_image_, canvas, CV_GRAY2BGR);

    for (int y = window_size; y < cart_size.height-window_size; y+=window_size/2) {
        for (int x = window_size; x < cart_size.width-window_size; x+=window_size/2) {
            cv::Rect window_roi = cv::Rect(x, y, window_size, window_size);
            float mask_mean = cv::mean(cart_image_mask(window_roi))[0]/255.0;

            if (mask_mean >= 0.5) {
                bool is_within_annotation;
                EvaluateImage(window_roi, cart_gradient_image_, is_within_annotation);
                cv::rectangle(canvas, window_roi, (is_within_annotation ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255)), 2);
            }
        }
    }

    mean_plot << pos_mean_vals_;
    mean_plot << neg_mean_vals_;
    mean_plot();

    stddev_plot << pos_stddev_vals_;
    stddev_plot << neg_stddev_vals_;
    stddev_plot();

    ShowScaledImage("sliding_window", canvas);
}

void CartesianProcessing::EvaluateImage(cv::Rect roi, const cv::Mat& src, bool& is_within_annotation) {
    is_within_annotation = false;
    std::map<std::string, cv::Mat>::iterator it = annotations_mask_.begin();

    while (it != annotations_mask_.end()) {
        cv::Mat mask;
        it->second.copyTo(mask);
        it++;

        float mask_mean = cv::mean(mask(roi))[0]/255.0;

        cv::Scalar mean_val;
        cv::Scalar stddev_val;
        cv::meanStdDev(src(roi), mean_val, stddev_val);

        if (mask_mean >= 0.5) {
            pos_mean_vals_.push_back(mean_val[0]);
            pos_stddev_vals_.push_back(stddev_val[0]);
            is_within_annotation = true;
        }
        else {
            neg_mean_vals_.push_back(mean_val[0]);
            neg_stddev_vals_.push_back(stddev_val[0]);
        }
    }
}

void CartesianProcessing::BuildCartesianMask(cv::InputArray src_arr, cv::OutputArray dst_arr) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(25, 25));
    cv::morphologyEx(src_arr, dst_arr, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
}

void CartesianProcessing::SaliencyMap() {
    cv::Mat cart_gradient_image_32f;
    cart_gradient_image_.convertTo(cart_gradient_image_32f, CV_32F, 1.0/255.0);
    ApplySalientMap(cart_gradient_image_32f, cart_saliency_map_, cart_image_mask_);
    ShowScaledImage("cart_saliency_map", cart_saliency_map_);
}

void CartesianProcessing::ApplySalientMap(cv::InputArray src_arr, cv::OutputArray dst_arr, cv::InputArray mask) {
    cv::Mat src = src_arr.getMat();

    CV_Assert(src.depth() == CV_32F);

    const int scale = 8;

    int width = src.size().width;
    int height = src.size().height;

    cv::Mat scale_image, scale_mask;
    cv::resize(src, scale_image, cv::Size(width / scale, height / scale));
    cv::resize(mask, scale_mask, cv::Size(width / scale, height / scale));

    cv::Mat saliency_map;
    features::saliency(scale_image, saliency_map, scale_mask);
    cv::normalize(saliency_map, saliency_map, 0, 1, cv::NORM_MINMAX);
    cv::resize(saliency_map, dst_arr, cv::Size(width, height));
}

void CartesianProcessing::Thresholding() {
    cv::Mat bin;
    ApplyThresholding(cart_saliency_map_, bin);
    ShowScaledImage("binary", bin);
}

void CartesianProcessing::ApplyThresholding(cv::InputArray src_arr, cv::OutputArray dst_arr) {
    CV_Assert(src_arr.getMat().depth() == CV_32F);
    cv::threshold(src_arr, dst_arr, 0.05, 1.0, CV_THRESH_BINARY);
}
