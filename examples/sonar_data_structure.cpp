#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_target_tracking/ImageUtils.hpp"
#include "base/test_config.h"

using namespace sonar_target_tracking;


void polar_to_cartesian_mapping(std::vector<float>& bins, std::vector<float>& bearings, uint32_t bin_count, uint32_t beam_count) {
    uint32_t width =  round(cos(bearings[bearings.size()-1] - M_PI_2) * bin_count) * 2.0;
    uint32_t height = bin_count;

    cv::Point2f origin = cv::Point2f(width / 2.0, height);

    std::vector<cv::Point2f> polar_to_cart(bin_count * beam_count);
    
    for (uint32_t bin = 0; bin < bin_count; bin++) {
        for (uint32_t beam = 0; beam < beam_count; beam++) {
            float t = bearings[beam] - M_PI_2;
            float r = bin;
            uint32_t polar_index = beam * bin_count + bin;
            polar_to_cart[polar_index] = base::MathUtil::to_cartesianf(t, r)  + origin;
        }
    }

    cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index kdtree(cv::Mat(polar_to_cart).reshape(1), indexParams);
    std::vector<float> query;

    uint32_t bin = bin_count - 10;
    uint32_t beam = beam_count - 10;
    uint32_t polar_index = beam * bin_count + bin;
    query.push_back(polar_to_cart[polar_index].x);
    query.push_back(polar_to_cart[polar_index].y);

    std::vector<int> indices;
    std::vector<float> dists;

    cv::flann::SearchParams params;
    kdtree.knnSearch(query, indices, dists, 5, params);

    printf("current: (%d %d)\n", beam, bin);
    for (int i = 0; i < indices.size(); i++) {
        beam = indices[i] / bin_count;
        bin = indices[i] % bin_count;
        polar_index = beam * bin_count + bin;
        cv::Point2f pt = polar_to_cart[polar_index];
        
        printf("neighbor: polar (%d %d) cartesian: (%f, %f)\n", beam, bin, pt.x, pt.y);
    }
    

    // std::vector<int> polar_to_cart(bin_count * beam_count, -1);
    // std::vector<int> cell_size(width * height, 0);
    // std::vector<int* > cart_to_polar(width * height);
    // 
    // for (uint32_t bin = 0; bin < bin_count; bin++) {
    //     for (uint32_t beam = 0; beam < beam_count; beam++) {
    //         float t = bearings[beam] - M_PI_2;
    //         float r = bin;
    //         cv::Point pos = base::MathUtil::to_cartesian(t, r) + origin;
    //         int polar_index = beam * bin_count + bin;
    //         int cart_index = pos.y * width + pos.x;
    //         polar_to_cart[polar_index] = cart_index;
    //         cell_size[cart_index]++;
    //     }
    // }
    // 
    // for (int polar_index = 0; polar_index < polar_to_cart.size(); polar_index++) {
    //     int cart_index = polar_to_cart[polar_index];
    // }
    // 
    // for (uint32_t bin = 0; bin < 5; bin++) {
    //     for (uint32_t beam = 0; beam < beam_count; beam++) {
    //         float t = bearings[beam] - M_PI_2;
    //         float r = bin;
    //         cv::Point pos = base::MathUtil::to_cartesian(t, r) + origin;
    //         uint32_t cart_index = pos.y * width + pos.x;
    //         std::cout << "cell_size: " << cell_size[cart_index] << std::endl;
    //         break;
    //     }
    // }
}


void weighted_polar_to_cartesian(std::vector<float>& bins, std::vector<float>& bearings, uint32_t bin_count, uint32_t beam_count) {
    uint32_t width =  round(cos(bearings[bearings.size()-1] - M_PI_2) * bin_count) * 2.0;
    uint32_t height = bin_count;

    cv::Point2f origin = cv::Point2f(width / 2.0, height);
    cv::Mat cart_image = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);

    cv::Mat radius = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
    cv::Mat thetas = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);

    float min_angle = bearings[0];
    float max_angle = bearings[bearings.size()-1];
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            float dx = origin.x - x;
            float dy = origin.y - y;
            float r = sqrt(dx * dx + dy * dy);
            float t = atan2(dy, dx) - M_PI_2;
            radius.at<float>(y, x) = r;
            thetas.at<float>(y, x) = t;
        }
    }

    for (uint32_t bin = 0; bin < bin_count-1; bin++) {
        for (uint32_t beam = 0; beam < beam_count-1; beam++) {

            float s0 = bins[beam*bin_count+bin+0];
            float s1 = bins[beam*bin_count+bin+1];
            float s2 = bins[(beam+1)*bin_count+bin+0];
            float s3 = bins[(beam+1)*bin_count+bin+1];

            float r0 = bin;
            float r1 = bin + 1;
            float t0 = bearings[beam+0];
            float t1 = bearings[beam+1];

            cv::Rect rc = base::MathUtil::arc_bounding_box(t0 - M_PI_2, t1 - M_PI_2, r0, r1, cv::Point(origin.x, origin.y));
    
            for (uint32_t y = rc.tl().y; y <= rc.br().y && y < height; y++) {
                for (uint32_t x = rc.tl().x; x <= rc.br().x && x < width; x++) {
                    float r = radius.at<float>(y, x);
                    float t = thetas.at<float>(y, x);

                    if (r <= r1 && r >= r0 && t >= t0 && t < t1) {
                        float v0 = s0 + (s1 - s0) * (r - r0);
                        float v1 = s2 + (s3 - s2) * (r - r0);
                        float v = v0 + (v1 - v0) * (t - t0) / (t1 - t0);
                        cart_image.at<float>(y, x) = v;
                    }
                }
            }
        }
    }

    image_utils::show_scale("cart_image", cart_image, 0.5);
    cv::waitKey();
}

int main(int argc, char const *argv[]) {
    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        base::samples::Sonar sample;

        while (stream.current_sample_index() < stream.total_samples()) {
            stream.next<base::samples::Sonar>(sample);
            std::vector<float> bearings = rock_util::Utilities::get_radians(sample.bearings);
            // weighted_polar_to_cartesian(sample.bins, bearings, sample.bin_count, sample.beam_count);
            polar_to_cartesian_mapping(sample.bins, bearings, sample.bin_count, sample.beam_count);
            break;
        }
    }

    return 0;
}
