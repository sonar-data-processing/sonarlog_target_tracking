#include <iostream>
#include <base/samples/Sonar.hpp>
#include <boost/filesystem.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_util/Converter.hpp"
#include "sonar_processing/ImageUtil.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "base/test_config.h"

using namespace sonar_processing;
using namespace rock_util;



std::vector<cv::Point2f> compute_centroid(std::vector<std::vector<cv::Point> > contours,
                      const std::vector<float>& bins,
                      const std::vector<float>& bearings,
                      const std::vector<int> beam_mapping,
                      uint32_t bin_count,
                      uint32_t beam_count,
                      cv::Size frame_size,
                      int mode = CV_RETR_EXTERNAL,
                      bool convex = false) {
    std::vector<std::vector<cv::Point> > convex_contours = preprocessing::convexhull(contours);
    cv::Mat mask = cv::Mat::zeros(cv::Size(bin_count, beam_count), CV_8UC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), CV_FILLED);
    cv::distanceTransform(mask, mask, CV_DIST_C, CV_DIST_MASK_PRECISE);
    cv::normalize(mask, mask, 0, 1, cv::NORM_MINMAX);
    cv::threshold(mask, mask, 0.95, 1, cv::THRESH_BINARY);
    cv::Mat polar_mask = sonar_util::Converter::convert2polar(image_util::mat2vector<float>(mask), bearings, bin_count, beam_count, frame_size.width, frame_size.height, beam_mapping);
    std::vector<std::vector<cv::Point> > polar_contours = preprocessing::find_contours(polar_mask, mode, convex);

    std::vector<cv::Point2f> centroids(polar_contours.size());
    float radius;
    cv::Point2f center;
    for (int i = 0; i < polar_contours.size(); i++) {
        cv::minEnclosingCircle(cv::Mat(polar_contours[i]), center, radius);
        if (radius > 5)  centroids.push_back(center);
    }
    return centroids;
}

std::vector<std::vector<cv::Point> > find_contours_polar(std::vector<std::vector<cv::Point> > contours,
                        const std::vector<float>& bins,
                        const std::vector<float>& bearings,
                        const std::vector<int> beam_mapping,
                        uint32_t bin_count,
                        uint32_t beam_count,
                        cv::Size frame_size,
                        int mode = CV_RETR_EXTERNAL,
                        bool convex = false) {

    cv::Mat mask = cv::Mat::zeros(cv::Size(bin_count, beam_count), CV_32FC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(1), CV_FILLED);
    cv::Mat polar_mask = sonar_util::Converter::convert2polar(image_util::mat2vector<float>(mask), bearings, bin_count, beam_count, frame_size.width, frame_size.height, beam_mapping);
    return preprocessing::find_contours(polar_mask, mode, convex);
}

void process_sample(base::samples::Sonar sample,
                    cv::Size frame_size,
                    std::vector<int> beam_mapping,
                    cv::VideoWriter segment_video,
                    cv::VideoWriter box_video) {

    cv::Mat src = cv::Mat(sample.bins).reshape(1, sample.beam_count);
    cv::Rect roi = preprocessing::calc_horiz_roi(src, 0.075);
    cv::Mat mat;
    src(roi).convertTo(mat, CV_8U, 255);

    preprocessing::remove_low_intensities_columns(mat, mat);

    std::vector<std::vector<cv::Point> > hi_contours = preprocessing::find_target_contours(mat);
    std::vector<std::vector<cv::Point> > shadow_contours = preprocessing::find_shadow_contours(mat);

    for( int i = 0; i < hi_contours.size(); i++ ) {
        for (int j = 0; j < hi_contours[i].size(); j++){
            hi_contours[i][j].x += roi.x;
        }
    }

    for( int i = 0; i < shadow_contours.size(); i++ ) {
        for (int j = 0; j < shadow_contours[i].size(); j++){
            shadow_contours[i][j].x += roi.x;
        }
    }

    std::vector<float> bearings_radians = Utilities::get_radians(sample.bearings);
    cv::Mat src_polar = sonar_util::Converter::convert2polar(sample.bins, bearings_radians, sample.bin_count, sample.beam_count, frame_size.width, frame_size.height, beam_mapping);
    cv::Mat segment_frame_polar, box_frame_polar;

    cv::cvtColor(src_polar, segment_frame_polar, CV_GRAY2BGR);
    cv::cvtColor(src_polar, box_frame_polar, CV_GRAY2BGR);

    std::vector<std::vector<cv::Point> > hi_contours_polar = find_contours_polar(hi_contours, sample.bins, bearings_radians, beam_mapping,
                                                                                 sample.bin_count, sample.beam_count, frame_size, CV_RETR_EXTERNAL, true);

    std::vector<std::vector<cv::Point> > shadow_contours_polar = find_contours_polar(shadow_contours, sample.bins, bearings_radians, beam_mapping,
                                                                                     sample.bin_count, sample.beam_count, frame_size, CV_RETR_LIST, false);



    std::vector<cv::Point2f> centroids = compute_centroid(hi_contours, sample.bins, bearings_radians, beam_mapping,
                                                          sample.bin_count, sample.beam_count, frame_size, CV_RETR_EXTERNAL, true);


    cv::drawContours(segment_frame_polar, hi_contours_polar, -1, cv::Scalar(0, 0, 255), 2);
    cv::drawContours(segment_frame_polar, shadow_contours_polar, -1, cv::Scalar(255, 0, 0), 2);

    for (int i = 0; i < centroids.size(); i++) {
        cv::circle(box_frame_polar, cv::Point(centroids[i].x, centroids[i].y), 10, cv::Scalar(0, 255, 0), 2, CV_AA);
    }

    for (int i = 0; i < hi_contours_polar.size(); i++) {

        if (hi_contours_polar[i].size() < 6) {
            continue;
        }

        cv::Mat points;
        cv::Mat(hi_contours_polar[i]).convertTo(points, CV_32S);
        cv::RotatedRect box = cv::minAreaRect(points);

        if (box.size.width < 10 && box.size.height < 10) {
            continue;
        }

        cv::Point2f vtx[4];
        box.points(vtx);
        for(int j = 0; j < 4; j++) cv::line(box_frame_polar, vtx[j], vtx[(j+1)%4], cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    int delay_scale = 5;
    int frame_id = 0;
    while (frame_id++ < delay_scale) {
        segment_video << segment_frame_polar;
        box_video << box_frame_polar;
    }
}

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);
    cv::VideoWriter segment_video;
    cv::VideoWriter box_video;

    boost::filesystem::create_directories(boost::filesystem::path("output"));

    std::string segment_video_path = "output/segment_result.mpg";
    std::string box_video_path = "output/box_result.mpg";

    uint32_t frame_height = 640;
    uint32_t frame_width = base::MathUtil::aspect_ratio_width(1.04047, frame_height);
    cv::Size frame_size(frame_width, frame_height);
    segment_video.open(segment_video_path, CV_FOURCC('P','I','M','1'), 25, frame_size, true);
    box_video.open(box_video_path, CV_FOURCC('P','I','M','1'), 25, frame_size, true);

    boost::filesystem::create_directories(boost::filesystem::path("output"));

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        if (stream.current_sample_index() >= stream.total_samples()) {
            continue;
        }


        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);

        std::vector<int> beam_mapping = sonar_util::Converter::generate_beam_mapping_from_cartesian(Utilities::get_radians(sample.bearings),
                                                                                                    sample.bin_count, sample.beam_count,
                                                                                                    frame_width, frame_height);
        process_sample(sample, frame_size, beam_mapping, segment_video, box_video);
        while (stream.current_sample_index() < stream.total_samples()) {
            stream.next<base::samples::Sonar>(sample);
            process_sample(sample, frame_size, beam_mapping, segment_video, box_video);
        }
    }

    return 0;
}
