#include <iostream>
#include <base/samples/Sonar.hpp>
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "sonar_processing/FrequencyDomain.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/ImageUtils.hpp"
#include "base/test_config.h"

using namespace sonar_processing;
using namespace sonar_processing::frequency_domain;

void central_frequencies_reject(const cv::Size& size, double D, int n, int center_distance, cv::OutputArray dstx, cv::OutputArray dsty) {
    uint32_t w = size.width;
    uint32_t h = size.height;

    int cx = w / 2;
    int cy = h / 2;

    cv::Mat filterx = cv::Mat(size, CV_8UC1);
    cv::Mat filtery = cv::Mat(size, CV_8UC1);

    filterx.setTo(255);
    filtery.setTo(255);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double dy = y - cy;
            double dx = x - cx;

            double rx = sqrt(dx * dx);
            double ry = sqrt(dy * dy);
            double r = sqrt(dx * dx + dy * dy);

            if (r > center_distance) {
                filterx.at<uchar>(y, x) = 1.0 / (1.0 + pow(D / rx, 2.0 * n)) * 255;
                filtery.at<uchar>(y, x) = 1.0 / (1.0 + pow(D / ry, 2.0 * n)) * 255;
            }
        }
    }

    filterx.convertTo(filterx, CV_32F, 1.0/255.0);
    filtery.convertTo(filtery, CV_32F, 1.0/255.0);

    cv::Mat to_mergex[] = {filterx, filterx};
    cv::merge(to_mergex, 2, dstx);

    cv::Mat to_mergey[] = {filtery, filtery};
    cv::merge(to_mergey, 2, dsty);
}

void test_ideal_lowpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::ideal_lowpass(src.size(), 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("ideal low pass filter", res);
}

void test_butterworth_lowpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::butterworth_lowpass(src.size(), 30, 1, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("butterworth low pass filter", res);
}

void test_gaussian_spatial_filter(const cv::Mat& src) {
    cv::Mat res;
    cv::GaussianBlur(src, res, cv::Size(15, 15), 0);
    cv::imshow("Spatial Gaussian Blur", res);
}

void test_gaussian_lowpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::gaussian_lowpass(src.size(), 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("gaussian low pass filter", res);
}

void test_ideal_highpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::ideal_highpass(src.size(), 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("ideal high pass filter", res);
}

void test_butterworth_highpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::butterworth_highpass(src.size(), 30, 1, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("butterworth high pass filter", res);
}

void test_gaussian_highpass_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::gaussian_highpass(src.size(), 200, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("gaussian high pass filter", res);
}

void test_ideal_bandreject_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::ideal_bandreject(src.size(), 50, 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("ideal band reject filter", res);
}

void test_butterworth_bandreject_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::butterworth_bandreject(src.size(), 50, 1, 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("butterworth band reject filter", res);
}

void test_gaussian_bandreject_filter(const cv::Mat& src) {
    cv::Mat tf;
    filters::gaussian_bandreject(src.size(), 50, 30, tf);
    cv::Mat res = tf.mul(src);
    dft::show_inverse("gaussian band reject filter", res);
}


void frequency_noise_removal(cv::InputArray src_arr, cv::OutputArray dst_arr) {
    cv::Mat src = src_arr.getMat();
    cv::Mat freq;
    dft::forward(src, freq);
    dft::show_spectrum("freq no shift", freq);
    dft::shift(freq);

    cv::Mat tf_x, tf_y;
    cv::Mat tf_bwlp;

    filters::butterworth_lowpass(freq.size(), 25, 2, tf_bwlp);
    central_frequencies_reject(freq.size(), 2, 1, 5, tf_x, tf_y);

    dft::show_spectrum("freq", freq);
    dft::show_spectrum("tf_x", tf_x);
    dft::show_spectrum("tf_y", tf_y);
    dft::show_spectrum("tf_bwlp", tf_bwlp);

    freq = tf_bwlp.mul(freq);
    freq = tf_x.mul(freq);
    freq = tf_y.mul(freq);

    dft::show_spectrum("freq_res", freq);

    cv::Mat mat;
    dft::inverse_abs(freq, mat);
    cv::normalize(mat, mat, 0, 1, cv::NORM_MINMAX);
    mat(cv::Rect(0, 0, src.cols, src.rows)).copyTo(dst_arr);
}

void run_filters(cv::Mat src) {
    cv::Mat srcf, noise_removal;
    src.convertTo(srcf, CV_32F, 1/255.0);
    frequency_noise_removal(srcf, noise_removal);
    noise_removal.convertTo(noise_removal, CV_8U, 255);
    cv::imshow("noise_removal", noise_removal);

    // cv::Mat sm;
    // cv::boxFilter(noise_removal, sm, CV_8U, cv::Size(15, 15));
    // cv::imshow("sm1", sm);

    cv::Mat sm;
    noise_removal.copyTo(sm);

    cv::Mat grad;
    preprocessing::gradient_filter(sm, grad);
    cv::normalize(grad, grad, 0, 255, cv::NORM_MINMAX);
    cv::imshow("grad1", grad);

    cv::boxFilter(sm, sm, CV_8U, cv::Size(80, 80));
    cv::imshow("sm2", sm);

    grad -= sm;
    cv::normalize(grad, grad, 0, 255, cv::NORM_MINMAX);
    cv::imshow("grad2", grad);

    cv::Mat bin;
    preprocessing::simple_thresholding(grad, bin, 0.2);
    cv::imshow("bin", bin);

    // cv::Mat fcomplex, icomplex, fmag, imag;
    // cv::Mat grad, src_sm;

    // cv::boxFilter(src_sm, src_sm, CV_8U, cv::Size(100, 100));
    // grad = src + (src - src_sm);
    // cv::normalize(grad, grad, 0, 255, cv::NORM_MINMAX);
    //
    // cv::boxFilter(grad, src_sm, CV_8U, cv::Size(15, 15));
    // preprocessing::gradient_filter(src_sm, grad);
    // cv::boxFilter(src_sm, src_sm, CV_8U, cv::Size(50, 50));
    //
    // cv::normalize(grad, grad, 0, 255, cv::NORM_MINMAX);
    // grad -= src_sm;
    // cv::normalize(grad, grad, 0, 255, cv::NORM_MINMAX);
    // cv::imshow("grad", grad);
    //
    //
    // cv::Mat srcf;
    // grad.convertTo(srcf, CV_32F, 1/255.0);
    // dft::forward(srcf, fcomplex);
    // dft::shift(fcomplex);
    //
    // dft::abs(fcomplex, fmag);
    // cv::normalize(fmag, fmag, 0, 1, cv::NORM_MINMAX);
    //
    // cv::imshow("magnitude spectrum", fmag);

    // test_butterworth_noise_removal(fcomplex);
    // test_gaussian_spatial_filter(src);
    // test_ideal_lowpass_filter(fcomplex);
    // test_butterworth_lowpass_filter(fcomplex);
    // test_gaussian_lowpass_filter(fcomplex);

    // test_ideal_highpass_filter(fcomplex);
    // test_butterworth_highpass_filter(fcomplex);
    // test_gaussian_highpass_filter(fcomplex);
    // test_ideal_bandreject_filter(fcomplex);
    // test_butterworth_bandreject_filter(fcomplex);
    // test_gaussian_bandreject_filter(fcomplex);
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
            cv::Mat src = cv::Mat(sample.bins).reshape(1, sample.beam_count);

            cv::Rect roi = preprocessing::calc_horiz_roi(src, 0.075);
            cv::Mat mat;
            src(roi).convertTo(mat, CV_8U, 255);
            preprocessing::remove_low_intensities_columns(mat, mat);
            run_filters(mat);
            cv::imshow("mat", mat);
            cv::waitKey();
        }
    }

    return 0;
}
