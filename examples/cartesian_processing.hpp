#include <iostream>
#include <map>
#include <vector>
#include <base/samples/Sonar.hpp>
#include "sonar_processing/SonarHolder.hpp"

class CartesianProcessing {

public:
    CartesianProcessing()
        : src_sonar_holder_(sonar_processing::SonarHolder())
        , annotations_(std::map<std::string, std::vector<cv::Point2f> >())
        , scale_factor_(0.5)
    {
    }

    CartesianProcessing(
        const sonar_processing::SonarHolder& sonar_holder)
        : src_sonar_holder_(sonar_holder)
        , annotations_(std::map<std::string, std::vector<cv::Point2f> >())
        , scale_factor_(0.5)
    {
    }

    CartesianProcessing(
        const sonar_processing::SonarHolder& sonar_holder,
        std::map<std::string, std::vector<cv::Point2f> > annotations )
        : src_sonar_holder_(sonar_holder)
        , annotations_(annotations)
        , scale_factor_(0.5)
    {
    }

    virtual void Process();

    void ImageScale(const cv::Mat& in, cv::Mat& out);

    void ShowScaledImage(const std::string& title, const cv::Mat& image);

    void ShowCartesianLineLimits();

    void DrawCartesianLineLimits(cv::Mat& out_image);

    void DetectRegionOfInterest();

    void BackgroundEstimation();

    void GradientFilter();

    void FrequencyNoiseRemoval();

    void BuildAnnotationsMask();

    void ShowAnnotationsMask();

    void ShowGradientAndAnnotation();

    void SaliencyMap();

    void Thresholding();

private:

    void point2f_to_point(const std::vector<cv::Point2f>& points2f, std::vector<cv::Point>& points) {
        if (points.empty()) points.resize(points2f.size());

        for (int i = 0; i < points2f.size(); i++) {
            points[i] = cv::Point(points2f[i].x, points2f[i].y);
        }
    }

    void BuildContourMask(const std::vector<cv::Point>& contour, cv::OutputArray dst_arr);

    void DrawContour(const std::vector<cv::Point>& contour, cv::OutputArray dst_arr, cv::Scalar color);

    void ProcessGradientImage();

    void EvaluateImage(cv::Rect roi, const cv::Mat& src, bool& is_within_annotation);

    void BuildCartesianMask(cv::InputArray src_arr, cv::OutputArray dst_arr);

    void ApplySalientMap(cv::InputArray src_arr, cv::OutputArray dst_arr, cv::InputArray mask);

    void ApplyGradientFilter(cv::InputArray src_arr, cv::OutputArray dst_arr, cv::InputArray mask);

    void ApplyThresholding(cv::InputArray src_arr, cv::OutputArray dst_arr);

    double scale_factor_;

    const sonar_processing::SonarHolder& src_sonar_holder_;
    const std::map<std::string, std::vector<cv::Point2f> > annotations_;

    std::map<std::string, cv::Mat> annotations_mask_;

    sonar_processing::SonarHolder dst_sonar_holder_;

    cv::Mat cart_background_image_;
    cv::Mat cart_image_mask_;
    cv::Mat cart_gradient_image_;
    cv::Mat cart_saliency_map_;

    int start_bin_;
    int final_bin_;

    std::vector<int> start_line_indices_;
    std::vector<int> final_line_indices_;

    std::vector<float> pos_mean_vals_;
    std::vector<float> neg_mean_vals_;

    std::vector<float> pos_stddev_vals_;
    std::vector<float> neg_stddev_vals_;
};
