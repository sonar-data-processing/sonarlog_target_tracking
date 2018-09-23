#include <iostream>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>


#define IMAGE_WIDTH     730*1.5
#define IMAGE_HEIGHT    422*1.5

#define BEAM_WIDTH          2.0944
#define BEAM_COUNT          8
#define BIN_COUNT           8
#define START_BEAM          -BEAM_WIDTH/2.0
#define BEAM_STEP           BEAM_WIDTH/(float)BEAM_COUNT
#define RAD2DEG(rad) ((rad) / M_PI * 180.0)
#define DEG2RAD(deg) ((deg) / 180.0 * M_PI)


cv::Point2f polar2cart(float theta, float radius) 
{
    float x = cos(theta-M_PI_2) * radius;
    float y = sin(theta-M_PI_2) * radius;
    return cv::Point2f(x, y);
}

cv::Point2f polar2cart(float theta, float radius, cv::Point2f origin, cv::Point2f scale, cv::Point2f pad = cv::Point2f()) 
{
    cv::Point2f pt = polar2cart(theta, radius);
    pt.x *= scale.x;
    pt.y *= scale.y;
    pt += origin;
    pt.x += pad.x/2;
    pt.y += pad.y/2;
    return pt;
}

cv::Point2f draw_ellipse(cv::Mat& img, cv::Point2f center, cv::Point2f pt, float t0, float t1, cv::Point2f pad = cv::Point2f())
{   

    center.x += pad.x/2;
    center.y += pad.y/2;

    float dx = pt.x-center.x;
    float dy = pt.y-center.y;
    float radius = sqrt(dx*dx + dy*dy);

    cv::Size2f axes = cv::Point2f(radius, radius);

    double startAngle = RAD2DEG(t0);
    double endAngle = RAD2DEG(t1);
    double angle = RAD2DEG(-M_PI_2);

    cv::ellipse(img,  center, axes, angle, startAngle, endAngle,  cv::Scalar(255, 0, 0), 2, CV_AA);
}


int main(int argc, char const **argv)
{

    std::cout << "Beam Width: " << RAD2DEG(BEAM_WIDTH) << " (" << BEAM_WIDTH  << ")" << std::endl;
    std::cout << "Beam Count: " << BEAM_COUNT << std::endl;
    std::cout << "Bin Count: " << BIN_COUNT << std::endl;
    std::cout << "Start Beam: " << RAD2DEG(START_BEAM) << " (" << START_BEAM  << ")" << std::endl;
    std::cout << "Beam Step: " << RAD2DEG(BEAM_STEP) << " (" << BEAM_STEP  << ")" << std::endl;

    cv::Size2f rectangular_size = cv::Size2f(cos(BEAM_WIDTH-M_PI_2)*(BIN_COUNT)*2.0, BIN_COUNT);
    cv::Size size = cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT);
    cv::Point2f origin = cv::Point2f(rectangular_size.width / 2, rectangular_size.height);


    cv::Point2f fs = cv::Point2f(
        size.width / (float)rectangular_size.width,
        size.height / (float)rectangular_size.height);

    origin.x *= fs.x;
    origin.y *= fs.y;

    int pad = 50;
    cv::Mat canvas = cv::Mat::zeros(cv::Size(size.width+pad, size.height+pad), CV_8UC3);
    canvas.setTo(cv::Scalar(255, 255, 255));

    for (uint32_t bin = 1; bin <= BIN_COUNT; bin++) {
        cv::Point2f pt = polar2cart(START_BEAM, bin, origin, fs, cv::Point(pad, pad));
        draw_ellipse(canvas, origin, pt, START_BEAM, START_BEAM+BEAM_WIDTH, cv::Point(pad, pad));
    }

    float theta = START_BEAM;
    for (uint32_t beam = 0; beam <= BEAM_COUNT; beam++) {
        cv::Point2f pt0 = polar2cart(theta, 0, origin, fs, cv::Point(pad, pad));
        cv::Point2f pt1 = polar2cart(theta, BIN_COUNT, origin, fs, cv::Point(pad, pad));
        cv::line(canvas, pt0, pt1, cv::Scalar(255, 0, 0), 2, CV_AA);
        theta += BEAM_STEP;
    }


    for (uint32_t bin = 0; bin < BIN_COUNT; bin++) {
        float theta = START_BEAM;
        for (uint32_t beam = 0; beam < BEAM_COUNT; beam++) {
            cv::Point2f pt0 = polar2cart(theta, bin, origin, fs, cv::Point(pad, pad));
            cv::Point2f pt1 = polar2cart(theta, bin+1, origin, fs, cv::Point(pad, pad));
            cv::Point2f pt2 = polar2cart(theta+BEAM_STEP, bin, origin, fs, cv::Point(pad, pad));
            cv::Point2f pt3 = polar2cart(theta+BEAM_STEP, bin+1, origin, fs, cv::Point(pad, pad));

            cv::circle(canvas, pt0, 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(canvas, pt1, 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(canvas, pt2, 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(canvas, pt3, 2, cv::Scalar(0, 0, 255), 2);

            theta += BEAM_STEP;

        }
    }

    cv::imshow("sonar", canvas);
    cv::waitKey();



    return 0;
}
