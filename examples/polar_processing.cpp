#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_processing/ImageUtils.hpp"
#include "sonar_processing/SonarHolder.hpp"
#include "sonar_processing/ROI.hpp"
#include "sonar_processing/Utils.hpp"
#include "sonar_processing/Filtering.hpp"
#include "base/test_config.h"

#define TITLE_CART_IMAGE_WINDOW         "polar_processing-cart_image"
#define TITLE_CART_IMAGE_WINDOW_RESULT  "polar_processing-cart_image-result"
#define TITLE_RAW_IMAGE_WINDOW          "polar_processing-raw_image"
#define TITLE_RAW_IMAGE_WINDOW_RESULT   "polar_processing-raw_image-result"

using namespace sonar_processing;

struct PolarProcessingContext  {
    PolarProcessingContext(const SonarHolder& sonar_holder)
        : sonar_holder(sonar_holder)
        , scale_factor(0.5)
        , start_bin(-1)
        , final_bin(-1)
    {
    }

    void initialize() {
        create_images();
        reset_canvas();
    }

    void create_images(){
        create_cart_image();
        create_raw_image();
    }

    void create_cart_image() {
        int w = sonar_holder.cart_size().width * scale_factor;
        int h = sonar_holder.cart_size().height * scale_factor;
        cv::resize(sonar_holder.cart_image(), cart_image, cv::Size(w, h));
    }

    void create_raw_image() {
        raw_image = sonar_holder.raw_image();
    }

    void reset_canvas() {
        reset_cart_image_canvas();
        reset_raw_image_canvas();
    }

    void reset_cart_image_canvas() {
        cv::cvtColor(cart_image, cart_image_canvas, CV_GRAY2BGR);
    }

    void reset_raw_image_canvas() {
        cv::cvtColor(raw_image, raw_image_canvas, CV_GRAY2BGR);
    }

    void print() {
        std::cout << "Start Bin: " << start_bin << std::endl;
        std::cout << "Final Bin: " << final_bin << std::endl;
    }

    void set_start_line_indices(const std::vector<int> line_indices) {
        start_line_indices = line_indices;
        sonar_holder.cart_points(start_line_indices, start_line_points);
    }

    void set_final_line_indices(const std::vector<int> line_indices) {
        final_line_indices = line_indices;
        sonar_holder.cart_points(line_indices, final_line_points);
    }

    void draw_start_line() {
        cv::line(cart_image_canvas, scale_to_image(start_line_points[0]), scale_to_image(start_line_points[start_line_points.size()-1]), cv::Scalar(0, 255, 255), 2);
        draw_line_points(start_line_points.begin(), start_line_points.end(), cv::Scalar(0, 255, 0));
        draw_raw_image_line(start_line_indices, cv::Scalar(0, 255, 0));
    }

    void draw_final_line() {
        cv::line(cart_image_canvas, scale_to_image(final_line_points[0]), scale_to_image(final_line_points[final_line_points.size()-1]), cv::Scalar(0, 255, 255), 2);
        draw_line_points(final_line_points.begin(), final_line_points.end(), cv::Scalar(0, 255, 0));
        draw_raw_image_line(final_line_indices, cv::Scalar(0, 255, 0));
    }

    void draw_line_points(std::vector<cv::Point2f>::iterator first, std::vector<cv::Point2f>::iterator last, cv::Scalar line_color) {
        if (first != last) {
            std::vector<cv::Point2f>::iterator it = first;
            while (it != (last-1)) {
                cv::Point2f pt0 = *it;
                cv::Point2f pt1 = *(it+1);
                cv::line(cart_image_canvas, scale_to_image(pt0), scale_to_image(pt1), line_color, 2);
                it++;
            }
        }
    }

    void draw_raw_image_line(const std::vector<int>& line, cv::Scalar line_color) {
        for (int i = 1; i < line.size(); i++) {
            cv::Point pt0 = cv::Point(sonar_holder.index_to_bin(line[i-1]), sonar_holder.index_to_beam(line[i-1]));
            cv::Point pt1 = cv::Point(sonar_holder.index_to_bin(line[i]), sonar_holder.index_to_beam(line[i]));
            cv::line(raw_image_canvas, pt0, pt1, line_color, 2);
        }
    }

    cv::Point2f scale_to_image(cv::Point2f pt) {
        return pt * scale_factor;
    }

    void draw() {
        draw_start_line();
        draw_final_line();
    }

    void show() {
        cv::imshow(TITLE_CART_IMAGE_WINDOW, cart_image);
        cv::imshow(TITLE_CART_IMAGE_WINDOW_RESULT, cart_image_canvas);
        cv::imshow(TITLE_RAW_IMAGE_WINDOW, raw_image);
        cv::imshow(TITLE_RAW_IMAGE_WINDOW_RESULT, raw_image_canvas);
    }

    std::vector<int> start_line_indices;
    std::vector<cv::Point2f> start_line_points;

    std::vector<int> final_line_indices;
    std::vector<cv::Point2f> final_line_points;

    float scale_factor;

    cv::Mat cart_image;
    cv::Mat cart_image_canvas;

    cv::Mat raw_image;
    cv::Mat raw_image_canvas;

    const SonarHolder& sonar_holder;

    int start_bin;
    int final_bin;
};

void PolarProcessing_initialize(PolarProcessingContext& context) {
    context.initialize();
}

void PolarProcessing_sonar_show(PolarProcessingContext& context, std::string prefix, const SonarHolder& sonar_holder) {
    cv::Size sz = cv::Size(sonar_holder.cart_size().width * context.scale_factor, sonar_holder.cart_size().height * context.scale_factor);
    cv::Mat im_cart;
    cv::resize(sonar_holder.cart_image(), im_cart, sz);
    std::string cartesian_title = prefix + "-cartesian";
    std::string polar_title = prefix + "-polar";
    cv::imshow(cartesian_title, im_cart);
    cv::imshow(polar_title, sonar_holder.raw_image());
}

void PolarProcessing_box_filter(PolarProcessingContext& context, SonarHolder& sonar_holder, int ksize = 3) {
    cv::Mat kernel = cv::Mat::ones(cv::Size(ksize, ksize), CV_32F) / (ksize * ksize);
    std::vector<float> bins;
    uint64_t start_time = sonar_processing::utils::now::milliseconds();
    filtering::filter2d(sonar_holder, bins, kernel);
    uint64_t delta_time = sonar_processing::utils::now::milliseconds()-start_time;
    printf("Box filter total time: %ld\n", delta_time);
    sonar_holder.ResetBins(bins);
}


void PolarProcessing_run(PolarProcessingContext& context) {
    // sonar_processing::roi::polar::bins_of_interest(context.sonar_holder, context.start_bin, context.final_bin);

    context.start_bin = 200;
    context.final_bin = context.sonar_holder.bin_count() - 200;

    std::vector<int> start_line_indices, final_line_indices;
    sonar_processing::basic_operations::line_indices_from_bin(context.sonar_holder, context.start_bin, start_line_indices);
    sonar_processing::basic_operations::line_indices_from_bin(context.sonar_holder, context.final_bin, final_line_indices);

    printf("start_bin size: %d\n", context.start_bin);
    printf("final_bin size: %d\n", context.final_bin);

    context.set_start_line_indices(start_line_indices);
    context.set_final_line_indices(final_line_indices);

    SonarHolder sonar_holder_roi;
    context.sonar_holder.CopyTo(sonar_holder_roi, context.start_line_indices, context.final_line_indices);

    PolarProcessing_box_filter(context, sonar_holder_roi, 7);
    PolarProcessing_sonar_show(context, "src", context.sonar_holder);
    PolarProcessing_sonar_show(context, "result", sonar_holder_roi);

    // context.draw();
    // context.print();
    // context.show();
}

int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    sonar_processing::SonarHolder sonar_holder;
    sonar_processing::PolarCartesianScanner polar_cartesian_scanner;

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

            if (sonar_holder.is_neighborhood_table_modified(sample.bin_count, sample.beam_count)) {
                std::cout << "Building neighborhood table..." << std::endl;
                uint64_t start_time = sonar_processing::utils::now::milliseconds();
                sonar_holder.BuildNeighborhoodTable(&polar_cartesian_scanner, sample.bin_count, sample.beam_count);
                uint64_t delta_time = sonar_processing::utils::now::milliseconds()-start_time;
                std::cout << "Neighborhood table built with success..." << std::endl;
                printf("Total time: %ld\n", delta_time);
            }

            PolarProcessingContext context(sonar_holder);
            PolarProcessing_initialize(context);
            PolarProcessing_run(context);
            if (stream.current_sample_index() == 1) cv::waitKey(); else cv::waitKey(25);

            stream.next<base::samples::Sonar>(sample);
        } while(stream.current_sample_index() < stream.total_samples());
    }
}
