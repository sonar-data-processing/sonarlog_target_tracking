#include <iostream>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include "Common.hpp"
#include "ArgumentParser.hpp"

#define SCALE 1.5
#define CART_IMAGE_WIDTH 730 * SCALE
#define CART_IMAGE_HEIGHT 422 * SCALE
#define POLAR_IMAGE_WIDTH 256 * SCALE
#define POLAR_IMAGE_HEIGHT 550 * SCALE
#define BEAM_WIDTH 2.0944
#define BEAM_COUNT 7
#define BIN_COUNT 8
#define LINE_SIZE   3
#define CIRCLE_SIZE 3
#define START_BEAM -BEAM_WIDTH / 2.0
#define BEAM_STEP BEAM_WIDTH / (float)BEAM_COUNT
#define RAD2DEG(rad) ((rad) / M_PI * 180.0)
#define DEG2RAD(deg) ((deg) / 180.0 * M_PI)

using namespace sonar_processing;
using namespace sonarlog_target_tracking;

struct Context {
    DatasetInfo dataset_info;
    SonarHolder sonar_holder;
};


cv::Point2f polar2cart(float theta, float radius)
{
    float x = cos(theta - M_PI_2) * radius;
    float y = sin(theta - M_PI_2) * radius;
    return cv::Point2f(x, y);
}

cv::Point2f polar2cart(float theta, float radius, cv::Point2f origin, cv::Point2f scale, cv::Point2f pad = cv::Point2f())
{
    cv::Point2f pt = polar2cart(theta, radius);
    pt.x *= scale.x;
    pt.y *= scale.y;
    pt += origin;
    pt.x += pad.x / 2;
    pt.y += pad.y / 2;
    return pt;
}

cv::Point2f draw_ellipse(cv::Mat &img, cv::Point2f center, cv::Point2f pt, float t0, float t1, cv::Point2f pad = cv::Point2f())
{

    center.x += pad.x / 2;
    center.y += pad.y / 2;

    float dx = pt.x - center.x;
    float dy = pt.y - center.y;
    float radius = sqrt(dx * dx + dy * dy);

    cv::Size2f axes = cv::Point2f(radius, radius);

    double startAngle = RAD2DEG(t0);
    double endAngle = RAD2DEG(t1);
    double angle = RAD2DEG(-M_PI_2);

    cv::ellipse(img, center, axes, angle, startAngle, endAngle, cv::Scalar(255, 0, 0), LINE_SIZE, CV_AA);
}

void sonarCartGen(cv::Mat &img, const SonarHolder& holder)
{
    std::cout << "Beam Width: " << RAD2DEG(BEAM_WIDTH) << " (" << BEAM_WIDTH << ")" << std::endl;
    std::cout << "Beam Count: " << BEAM_COUNT << std::endl;
    std::cout << "Bin Count: " << BIN_COUNT << std::endl;
    std::cout << "Start Beam: " << RAD2DEG(START_BEAM) << " (" << START_BEAM << ")" << std::endl;
    std::cout << "Beam Step: " << RAD2DEG(BEAM_STEP) << " (" << BEAM_STEP << ")" << std::endl;

    std::cout << "Beam Width: " << RAD2DEG(BEAM_WIDTH) << " (" << BEAM_WIDTH << ")" << std::endl;
    std::cout << "Beam Count: " << BEAM_COUNT << std::endl;
    std::cout << "Bin Count: " << BIN_COUNT << std::endl;
    std::cout << "Start Beam: " << RAD2DEG(START_BEAM) << " (" << START_BEAM << ")" << std::endl;
    std::cout << "Beam Step: " << RAD2DEG(BEAM_STEP) << " (" << BEAM_STEP << ")" << std::endl;

    float bin_count = BIN_COUNT;
    float first_beam = holder.first_beam_value();
    float last_beam = holder.last_beam_value();
    float beam_width = (last_beam - first_beam);
    float beam_step = beam_width / BEAM_COUNT;

    cv::Size2f size_ref =
          cv::Size2f(cos(beam_width - M_PI_2) * (BIN_COUNT)*2.0, BIN_COUNT);
    cv::Size size = holder.cart_size();
    cv::Point2f origin = cv::Point2f(size_ref.width / 2, size_ref.height);

    cv::Point2f fs = cv::Point2f(
        size.width / (float)size_ref.width,
        size.height / (float)size_ref.height);

    origin.x *= fs.x;
    origin.y *= fs.y;

    int pad = 50;
    cv::Mat cart_image;
    // cv::cvtColor(holder.cart_image(), cart_image, CV_GRAY2BGR);
    // cart_image.convertTo(cart_image, CV_8U, 255.0);
    cart_image = cv::Mat(holder.cart_image().size(), CV_8UC3);
    cart_image.setTo(cv::Scalar(255, 255, 255));

    cv::Mat canvas = cv::Mat::zeros(cv::Size(size.width + pad, size.height + pad), CV_8UC3);
    canvas.setTo(cv::Scalar(255, 255, 255));

    cv::Rect r = cv::Rect(pad/2, pad/2, cart_image.size().width, cart_image.size().height);
    cart_image.copyTo(canvas(r), holder.cart_image_mask());

    float d = 0.05;
    for (float bin = 1.0+d+2; bin <= bin_count+d-1; bin+=1.0)
    {
        cv::Point2f pt =
            polar2cart(first_beam+beam_step*2, bin, origin, fs, cv::Point(pad, pad));

        draw_ellipse(
            canvas, origin, pt, first_beam+beam_step*2, first_beam+beam_width-beam_step*2, cv::Point(pad, pad));
    }

    float theta = first_beam+beam_step*2;
    for (uint32_t beam = 2; beam <= BEAM_COUNT-2; beam++)
    {
        cv::Point2f pt0 = polar2cart(theta, 3, origin, fs, cv::Point(pad, pad));
        cv::Point2f pt1 = polar2cart(theta, bin_count+d-1, origin, fs, cv::Point(pad, pad));
        cv::line(canvas, pt0, pt1, cv::Scalar(255, 0, 0), LINE_SIZE, CV_AA);
        theta += beam_step;
    }

    img = canvas;
}

void sonarPolarGen(cv::Mat &img, const SonarHolder& holder)
{

    cv::Mat raw_image = holder.raw_image();
    cv::flip(raw_image, raw_image, 1);
    raw_image = raw_image.t();

    cv::Size size = cv::Size(POLAR_IMAGE_WIDTH, POLAR_IMAGE_HEIGHT);
    cv::resize(raw_image, raw_image, size);
    cv::cvtColor(raw_image, raw_image, CV_GRAY2BGR);
    raw_image.convertTo(raw_image, CV_8U, 255.0);

    int pad = 50;
    cv::Mat canvas = cv::Mat::zeros(cv::Size(size.width + pad, size.height + pad), CV_8UC3);
    canvas.setTo(cv::Scalar(255, 255, 255));

    cv::Rect r = cv::Rect(pad/2, pad/2, raw_image.size().width, raw_image.size().height);
    raw_image.copyTo(canvas(r));

    float beamSize = POLAR_IMAGE_WIDTH / BEAM_COUNT;
    float binSize = POLAR_IMAGE_HEIGHT / BIN_COUNT;

    for (size_t beam = 0; beam <= BEAM_COUNT; beam++)
    {
        float x = beam * beamSize + pad / 2;
        cv::Point2f pt0 = cv::Point2f(x, pad / 2);
        cv::Point2f pt1 = cv::Point2f(x, size.height + pad / 2);
        cv::line(canvas, pt0, pt1, cv::Scalar(255, 0, 0), LINE_SIZE, CV_AA);
    }

    for (size_t bin = 0; bin <= BIN_COUNT; bin++)
    {
        float y = bin * binSize + pad / 2;
        cv::Point2f pt0 = cv::Point2f(pad / 2, y);
        cv::Point2f pt1 = cv::Point2f(size.width + pad / 2, y);
        cv::line(canvas, pt0, pt1, cv::Scalar(255, 0, 0), LINE_SIZE, CV_AA);
    }

    img = canvas;
}

void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data)
{
  Context *pContext = reinterpret_cast<Context*>(user_data);


  if (pContext->dataset_info.preprocessing_settings().image_max_size != cv::Size(-1, -1)) {
        sonarlog_target_tracking::common::load_sonar_holder(
            sample,
            pContext->sonar_holder,
            cv::Size(CART_IMAGE_WIDTH, CART_IMAGE_HEIGHT));
    }
    else {
        sonarlog_target_tracking::common::load_sonar_holder(sample, pContext->sonar_holder);
    }

    cv::Mat source_mask = pContext->sonar_holder.cart_image_mask();
    cv::Mat cart;
    sonarCartGen(cart, pContext->sonar_holder);

    cv::Mat polar;
    sonarPolarGen(polar, pContext->sonar_holder);

    cv::imshow("cart", cart);
    cv::imshow("polar", polar);

    if ('s' == (char)cv::waitKey()){
      std::cout << "Save images..." << std::endl;
      cv::imwrite("cart.png", cart);
      cv::imwrite("polar.png", polar);
    }
}

int main(int argc, char **argv)
{
    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    Context context;
    context.dataset_info =  DatasetInfo(argument_parser.dataset_info_filename());
    DatasetInfo(argument_parser.dataset_info_filename());
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> positive_entries = context.dataset_info.positive_entries();

    for (size_t i=0; i<positive_entries.size(); i++) {
        common::exec_samples_from_dataset_entry(
          positive_entries[i],
          sample_receiver_callback,
          &context);
    }

    // cv::Mat cart, polar;
    // sonarCartGen(cart);
    // sonarPolarGen(polar);

    // cv::Mat output;
    // cv::hconcat(polar, cart, output);
    // cv::imshow("sonargen", output);
    // cv::imwrite("sonargen.png", output);
    // cv::waitKey();
    return 0;
}
