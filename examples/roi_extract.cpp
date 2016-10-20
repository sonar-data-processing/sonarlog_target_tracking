#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_target_tracking/ImageUtils.hpp"
#include "sonar_target_tracking/SonarHolder.hpp"
#include "sonar_target_tracking/ROI.hpp"
#include "base/test_config.h"

#define WINDOW_TITLE           "roi_extract-source_image"
#define WINDOW_TITLE_RESULT    "roi_extract-result_image"

using namespace sonar_target_tracking;

struct ROIExtractContext  {
    ROIExtractContext(const SonarHolder& sonar_holder)
        : sonar_holder(sonar_holder)
        , scale_factor(0.5)
        , start_bin(-1)
        , final_bin(-1)
    {
    }

    void create_rectangular_image() {
        int w = sonar_holder.cart_size().width * scale_factor;
        int h = sonar_holder.cart_size().height * scale_factor;
        cv::resize(sonar_holder.cart_image(), cart_image, cv::Size(w, h));
    }

    void reset_canvas() {
        reset_rectangular_image_canvas();
    }

    void reset_rectangular_image_canvas() {
        cv::cvtColor(cart_image, cart_canvas_image, CV_GRAY2BGR);
    }

    void print() {
        std::cout << "Start Bin: " << start_bin << std::endl;
        std::cout << "Final Bin: " << final_bin << std::endl;
    }

    void set_start_line_indices(const std::vector<int> line_indices) {
        start_line_indices = line_indices;
        sonar_holder.cart_points(line_indices, start_line_points);
    }

    void set_final_line_indices(const std::vector<int> line_indices) {
        final_line_indices = line_indices;
        sonar_holder.cart_points(line_indices, final_line_points);
    }

    void draw_start_line() {
        cv::line(cart_canvas_image, scale_to_image(start_line_points[0]), scale_to_image(start_line_points[start_line_points.size()-1]), cv::Scalar(0, 255, 255), 2);
        draw_line_points(start_line_points.begin(), start_line_points.end(), cv::Scalar(0, 255, 0));
    }

    void draw_final_line() {
        cv::line(cart_canvas_image, scale_to_image(final_line_points[0]), scale_to_image(final_line_points[final_line_points.size()-1]), cv::Scalar(0, 255, 255), 2);
        draw_line_points(final_line_points.begin(), final_line_points.end(), cv::Scalar(0, 255, 0));
    }

    void draw_line_points(std::vector<cv::Point2f>::iterator first, std::vector<cv::Point2f>::iterator last, cv::Scalar line_color) {
        if (first != last) {
            std::vector<cv::Point2f>::iterator it = first;
            while (it != (last-1)) {
                cv::Point2f pt0 = *it;
                cv::Point2f pt1 = *(it+1);
                cv::line(cart_canvas_image, scale_to_image(pt0), scale_to_image(pt1), line_color, 2);
                it++;
            }
        }
    }

    cv::Point2f scale_to_image(cv::Point2f pt) {
        return pt * scale_factor;
    }

    std::vector<int> start_line_indices;
    std::vector<cv::Point2f> start_line_points;

    std::vector<int> final_line_indices;
    std::vector<cv::Point2f> final_line_points;

    float scale_factor;
    cv::Mat cart_image;
    cv::Mat cart_canvas_image;
    const SonarHolder& sonar_holder;

    int start_bin;
    int final_bin;
};

void ROIExtract_initialize(ROIExtractContext& context) {
    context.create_rectangular_image();
    context.reset_canvas();
}

void ROIExtract_run(ROIExtractContext& context) {

    sonar_target_tracking::roi::bins_of_interest(context.sonar_holder, context.start_bin, context.final_bin);

    std::vector<int> start_line_indices, final_line_indices;
    sonar_target_tracking::basic_operations::line_indices_from_bin(context.sonar_holder, context.start_bin, start_line_indices);
    sonar_target_tracking::basic_operations::line_indices_from_bin(context.sonar_holder, context.final_bin, final_line_indices);

    context.set_start_line_indices(start_line_indices);
    context.set_final_line_indices(final_line_indices);

    context.draw_start_line();
    context.draw_final_line();

    context.print();

    cv::imshow(WINDOW_TITLE, context.cart_image);
    cv::imshow(WINDOW_TITLE_RESULT, context.cart_canvas_image);

    cv::waitKey(25);
}

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    sonar_target_tracking::SonarHolder sonar_holder;

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);

        do {

            sonar_holder.Reset(sample.bins,
                rock_util::Utilities::get_radians(sample.bearings),
                sample.beam_width.getRad(),
                sample.bin_count,
                sample.beam_count);
                stream.next<base::samples::Sonar>(sample);

            ROIExtractContext context(sonar_holder);
            ROIExtract_initialize(context);
            ROIExtract_run(context);
            // break;

        } while(stream.current_sample_index() < stream.total_samples());
        // break;
        // cv::waitKey();
    }
}
