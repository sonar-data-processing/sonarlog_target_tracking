#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <base/samples/Sonar.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/ml/ml.hpp>
#include "sonar_processing/HogDescriptorViz.hpp"
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/SonarHolder.hpp"
#include "sonar_processing/SonarImagePreprocessing.hpp"
#include "sonar_processing/Utils.hpp"
#include "sonarlog_annotation/AnnotationFileReader.hpp"
#include "sonarlog_target_tracking/Common.hpp"
#include "sonarlog_target_tracking/DetectionStats.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"

using namespace sonar_processing;
using namespace sonarlog_annotation;

static boost::filesystem::path g_log_file_path;

class LinearSVM : public cv::SVM {

public:
    void get_detector(std::vector<float>& support_vector);
};

void LinearSVM::get_detector(std::vector<float>& detector) {
    const CvSVMDecisionFunc* df = decision_func;
    const double* alphas = df->alpha;
    double rho = df->rho;

    int sv_count = get_support_vector_count();
    int var_count = get_var_count();

    detector.clear();
    detector.resize(var_count, 0);

    for (int i = 0; i < sv_count; i++) {
        const float* v = get_support_vector(i);
        float alpha = alphas[i];
        for (int j = 0; j < var_count; j++,v++) {
            detector[j] += (alpha) * (*v);
        }
    }
    detector.push_back((float)-rho);
}


void compute_hog(const cv::Mat& src, std::vector<cv::Mat> & gradient_lst, const cv::Size & size, bool show_hogdescriptor=false) {
    cv::HOGDescriptor hog;
    hog.winSize = size;

    cv::Mat gray;
    std::vector<cv::Point> location;
    std::vector<float> descriptors;

    hog.compute(src, descriptors, cv::Size(8, 8), cv::Size(0, 0), location);
    gradient_lst.push_back(cv::Mat(descriptors).clone());

    if (show_hogdescriptor) {
        cv::Mat rgb;
        cv::cvtColor(src, rgb, CV_GRAY2BGR);
        cv::imshow("gradient", get_hogdescriptor_visu(rgb, descriptors, size));
    }
}

void prepare_hog_input_positive(const cv::Mat& src, const cv::Mat& mask, cv::Mat& dst, cv::Size hog_win_size) {
    float scale_factor = 0.3;

    cv::Mat scaled_image;
    cv::resize(src, scaled_image, cv::Size(), scale_factor, scale_factor);

    cv::Mat scaled_mask;
    cv::resize(mask, scaled_mask, cv::Size(), scale_factor, scale_factor);

    cv::Rect bounding_rect = image_util::get_bounding_rect(scaled_mask);

    cv::Mat target_mask;
    scaled_mask(bounding_rect).convertTo(target_mask, CV_32F, 1.0/255.0);

    dst = scaled_image(bounding_rect).mul(target_mask);

    cv::resize(dst, dst, hog_win_size);
    dst.convertTo(dst, CV_8U, 255.0);
}

void compute_hog_positive(const cv::Mat& src, const cv::Mat& annotation_mask, std::vector<cv::Mat>& gradient_list_positive, bool show_hogdescriptor=false) {
    cv::Size hog_win_size = cv::Size(208, 48);
    cv::Mat hog_input_positive;
    prepare_hog_input_positive(src, annotation_mask, hog_input_positive, hog_win_size);
    compute_hog(hog_input_positive, gradient_list_positive, hog_win_size, show_hogdescriptor);
}

void compute_hog_negative(const cv::Mat& src, const cv::Mat& mask, const cv::Mat annotation_mask, std::vector<cv::Mat>& gradient_list_negative, bool show_hogdescriptor=false) {
    cv::Size hog_win_size = cv::Size(208, 48);

    image_util::dilate(mask, mask, cv::Size(15, 15), 2);

    // set region of interest
    cv::Rect bounding_rect = image_util::get_bounding_rect(mask);

    // set region of interest
    cv::Rect annotation_bounding_rect = image_util::get_bounding_rect(annotation_mask);

    float scale_factor = 0.3;
    cv::Mat scaled;
    cv::resize(src(bounding_rect), scaled, cv::Size(0, 0), scale_factor, scale_factor);

    cv::Mat scaled_mask;
    cv::resize(mask(bounding_rect), scaled_mask, cv::Size(0, 0), scale_factor, scale_factor);

    cv::Mat scaled_annotation_mask;
    cv::resize(annotation_mask(bounding_rect), scaled_annotation_mask, cv::Size(0, 0), scale_factor, scale_factor);

    scaled_mask.setTo(0, scaled_annotation_mask);
    scaled.convertTo(scaled, CV_8U, 255.0);

    cv::Size sz = scaled.size();
    for (int y = 0; y < sz.height; y+=hog_win_size.height) {
        if (((sz.height-y)/(float)sz.height) < 0.1) continue;

        int yy = ((y+hog_win_size.height)>=sz.height) ? sz.height-hog_win_size.height : y;
        yy = std::max<int>(yy, 0);
        if (yy < 0) continue;

        for (int x = 0; x < sz.width; x+=hog_win_size.width) {
            if (((sz.width-x)/(float)sz.width) < 0.1) continue;
            int xx = ((x+hog_win_size.width)>=sz.width) ? sz.width-hog_win_size.width : x;
            if (xx < 0) continue;

            cv::Rect rc = cv::Rect(xx, yy, hog_win_size.width, hog_win_size.height);
            double m = cv::mean(scaled_annotation_mask(rc))[0]/255.0;

            if (m < 0.2) {
                cv::Mat hog_input_negative;
                scaled(rc).copyTo(hog_input_negative);
                compute_hog(hog_input_negative, gradient_list_negative, hog_win_size, show_hogdescriptor);
            }
        }
    }
}

void orientation_normalize(const cv::Mat& src, const cv::Mat& mask, const cv::Mat& annotation_mask, const cv::RotatedRect& bbox,
                          cv::Mat& rotated, cv::Mat& rotated_mask, cv::Mat& rotated_annotation_mask, double& rotated_angle) {

    // rotate images to horizontal position
    cv::Point2f center = cv::Point2f(src.cols/2, src.rows/2);

    rotated_angle = (bbox.size.width>=bbox.size.height) ? bbox.angle : bbox.angle+90;

    image_util::rotate(src, rotated, rotated_angle, center);
    image_util::rotate(mask, rotated_mask, rotated_angle, center);
    image_util::rotate(annotation_mask, rotated_annotation_mask, rotated_angle, center);
}

void perform_preprocessing(const SonarHolder& sonar_holder, cv::Mat& preprocessed_image, cv::Mat& preprocessed_mask, int sample_number=-1) {

    // load preprocessed image file
    std::string image_filename;
    std::string mask_filename;
    if (sample_number != -1) {
        std::stringstream ss;
        ss << g_log_file_path.string();
        ss << "/";
        ss << sample_number;

        image_filename = ss.str()+".png";
        mask_filename = ss.str()+"_mask.png";

        if (boost::filesystem::exists(image_filename) &&
            boost::filesystem::exists(mask_filename)) {
            preprocessed_image = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);
            preprocessed_mask = cv::imread(mask_filename, CV_LOAD_IMAGE_GRAYSCALE);
            preprocessed_image.convertTo(preprocessed_image, CV_32F, 1.0/255.0);
            preprocessed_mask.convertTo(preprocessed_mask, CV_32F, 1.0/255.0);
            return;
        }
    }

    // process the cart image
    SonarImagePreprocessing sonar_image_processing;
    sonar_image_processing.set_clahe_final_clip_limit(2.0);
    sonar_image_processing.set_mean_filter_ksize(5);
    sonar_image_processing.set_mean_difference_filter_ksize(75);
    sonar_image_processing.set_median_blur_filter_ksize(5);
    sonar_image_processing.set_saliency_map_block_count(12);
    sonar_image_processing.set_saliency_map_thresh_factor(0.3);
    sonar_image_processing.set_saliency_map_scale_factor(0.25);
    sonar_image_processing.Apply(sonar_holder, preprocessed_image, preprocessed_mask, 0.5);

    // save preprocessed image
    if (!image_filename.empty() && !mask_filename.empty()) {
        cv::Mat out_image, out_mask;
        preprocessed_image.convertTo(out_image, CV_8U, 255.0);
        preprocessed_mask.convertTo(out_mask, CV_8U, 255.0);
        cv::imwrite(image_filename, out_image);
        cv::imwrite(mask_filename, out_mask);
    }
}

void hog_preprocessing(const SonarHolder& sonar_holder, const std::vector<cv::Point>& annotations, const cv::Mat& annotation_mask,
                       cv::Mat& dst_image, cv::Mat& dst_mask, cv::Mat& dst_annotation_mask, double& rotated_angle, int sample_number=-1) {

    // perform preprocessing
    cv::Mat preprocessed_image, preprocessed_mask;
    perform_preprocessing(sonar_holder, preprocessed_image, preprocessed_mask, sample_number);

    // get annotation rotated rect
    cv::RotatedRect box = cv::minAreaRect(annotations);
    orientation_normalize(preprocessed_image, preprocessed_mask, annotation_mask, box, dst_image, dst_mask, dst_annotation_mask, rotated_angle);
}

bool validate_hog_preprocessing(const cv::Mat& src, const cv::Mat& mask, const cv::Mat& annotation_mask, float intersection_thresh=0.7) {
    cv::Mat mask_8u;
    cv::Mat annotation_mask_8u;

    mask.convertTo(mask_8u, CV_8U, 255.0);
    annotation_mask.convertTo(annotation_mask_8u, CV_8U, 255.0);

    cv::Mat intersection_mask;
    cv::bitwise_and(mask_8u, annotation_mask_8u, intersection_mask);

    double annotation_area = cv::sum(annotation_mask)[0]/255.0;
    double intersection_area = cv::sum(intersection_mask)[0]/255.0;

    return (intersection_area/annotation_area)>=intersection_thresh;
}

void load_training_data(const std::vector<base::samples::Sonar>& samples, const std::vector<std::vector<cv::Point> >& annotations, int start_sample, int train_sample_count,
                        std::vector<cv::Mat>& gradient_positive, std::vector<cv::Mat>& gradient_negative) {

    SonarHolder sonar_holder;

    size_t final_sample_index = start_sample+train_sample_count;

    for (size_t sample_index = start_sample; sample_index<samples.size() && sample_index<final_sample_index; sample_index++) {
        std::cout << "Training sample: " << sample_index << std::endl;

        base::samples::Sonar sample = samples[sample_index];

        // load sonar holder data structure
        sonarlog_target_tracking::common::load_sonar_holder(sample, sonar_holder);

        // adjust annotation to fit the preprocessed image
        std::vector<cv::Point> annotation_points = annotations[sample_index];
        cv::Mat annotation_mask;
        sonarlog_target_tracking::common::adjust_annotation(sonar_holder.cart_size(), annotation_points, annotation_points, annotation_mask);

        // apply a dilate to increase the annotation area
        image_util::dilate(annotation_mask, annotation_mask, cv::Size(15, 15), 2);

        // perform hog preprocessing
        cv::Mat hog_input_image, hog_input_mask, hog_annotation_mask;
        double rotated_angle;
        hog_preprocessing(sonar_holder, annotation_points, annotation_mask,
                          hog_input_image, hog_input_mask, hog_annotation_mask,
                          rotated_angle, sample_index);

        if (validate_hog_preprocessing(hog_input_image, hog_input_mask, hog_annotation_mask)) {
            // compute hog positive descriptors
            compute_hog_positive(hog_input_image, hog_annotation_mask, gradient_positive);
        }

        // compute hog negative descriptors
        compute_hog_negative(hog_input_image, hog_input_mask, hog_annotation_mask, gradient_negative);

    }
}

void prepare_training_data(const std::vector<cv::Mat>& positive, const std::vector<cv::Mat>& negative, std::vector<int>& labels, cv::Mat& train_data) {
    labels.clear();
    labels.insert(labels.end(), positive.size(), +1);
    labels.insert(labels.end(), negative.size(), -1);

    std::vector<cv::Mat> train_samples;
    train_samples.insert(train_samples.end(), positive.begin(), positive.end());
    train_samples.insert(train_samples.end(), negative.begin(), negative.end());

    const int rows = (int)train_samples.size();
    const int cols = (int)std::max(train_samples[0].cols, train_samples[0].rows);

    train_data = cv::Mat(rows, cols, CV_32FC1);

    cv::Mat train_sample;
    std::vector<cv::Mat>::const_iterator itr = train_samples.begin();
    std::vector<cv::Mat>::const_iterator end = train_samples.end();
    for(int i = 0 ; itr != end ; ++itr, ++i)
    {
        itr->copyTo(train_sample);
        CV_Assert(train_sample.cols == 1 || train_sample.rows == 1);

        if(train_sample.cols == 1) {
            cv::transpose(train_sample, train_sample);
        }

        train_sample.copyTo(train_data.row(i));
    }
}

void perform_svm_train(const std::vector<int> labels, const cv::Mat& train_data, const std::string& train_filepath) {
    cv::SVM svm;

    // Set up SVM's parameters
    cv::SVMParams params;
    params.coef0 = 0.0;
    params.degree = 3.0;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 1000, 1e-6);
    params.gamma = 0;
    params.kernel_type = CvSVM::LINEAR;
    params.nu = 0.5;
    params.p = 0.1;
    params.C = 0.01;
    params.svm_type = CvSVM::EPS_SVR;
    svm.train(train_data, cv::Mat(labels), cv::Mat(), cv::Mat(), params);
    svm.save(train_filepath.c_str());
}

void find_contour(const cv::Mat& src, std::vector<std::vector<cv::Point> >& contours) {
    cv::Mat im;
    src.copyTo(im);
    cv::findContours(im, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
}

void build_locations_masks(const std::vector<cv::Rect> locations,
                          cv::Size input_size, cv::Size output_size, cv::Point offset, float scale_factor, float rotated_angle,
                          std::vector<cv::Mat>& masks, std::vector<cv::RotatedRect>& boxs, bool only_first_location=false) {

    if (!locations.empty()) {
        cv::Point center = cv::Point(input_size.width/2, input_size.height/2);
        std::vector<cv::Rect>::const_iterator it;
        for(it = locations.begin(); it != locations.end(); ++it ) {
            cv::Mat mask = cv::Mat::zeros(input_size, CV_8UC1);
            cv::Rect rc;
            rc.x = (*it).x/scale_factor+offset.x/scale_factor;
            rc.y = (*it).y/scale_factor+offset.y/scale_factor;
            rc.width = (*it).width/scale_factor;
            rc.height = (*it).height/scale_factor;

            cv::rectangle(mask, rc, cv::Scalar(255), CV_FILLED);
            image_util::rotate(mask, mask, -rotated_angle, center, output_size);

            std::vector<cv::Point> contours = preprocessing::find_biggest_contour(mask);
            cv::RotatedRect box = cv::minAreaRect(contours);

            boxs.push_back(box);
            masks.push_back(mask);

            if (only_first_location){
                return;
            }
        }
    }
}

void draw_locations(const cv::Mat& src, cv::Mat& dst, const std::vector<cv::RotatedRect> loc_boxs) {
    cv::cvtColor(src, dst, CV_GRAY2BGR);
    if (!loc_boxs.empty()) {
        std::vector<cv::RotatedRect>::const_iterator it = loc_boxs.begin();
        for(; it != loc_boxs.end(); ++it ) {
            // get annotation rotated rect
            image_util::draw_rotated_rect(dst, cv::Scalar(0, 255, 0), *it);
            cv::circle(dst, (*it).center, 10, cv::Scalar(0, 255, 0), CV_FILLED);
        }
    }
}

void perform_svm_test(const std::vector<base::samples::Sonar>& samples,
                      const std::vector<std::vector<cv::Point> >& annotations,
                      const std::string& train_filepath,
                      std::vector<std::vector<double> >& accuracy_levels,
                      std::vector<std::vector<double> >& classifier_weights,
                      std::vector<std::vector<cv::RotatedRect> >& detector_results,
                      std::vector<cv::Size>& frame_sizes,
                      int start_sample=0) {

    cv::Size hog_win_size = cv::Size(208, 48);

    LinearSVM svm;
    svm.load(train_filepath.c_str());

    std::vector<float> hog_detector;
    svm.get_detector(hog_detector);

    cv::HOGDescriptor hog;
    hog.winSize=hog_win_size;
    hog.setSVMDetector(hog_detector);

    SonarHolder sonar_holder;

    accuracy_levels.resize(samples.size());
    classifier_weights.resize(samples.size());
    detector_results.resize(samples.size());
    frame_sizes.resize(samples.size(), cv::Size(-1, -1));

    printf("\033[s");
    for (size_t sample_index = start_sample; sample_index < samples.size(); sample_index++) {
        printf("\033[u");
        printf("Testing sample: %ld ", sample_index);
        fflush(stdout);

        base::samples::Sonar sample = samples[sample_index];

        // load sonar holder data structure
        sonarlog_target_tracking::common::load_sonar_holder(sample, sonar_holder);

        // set the frame size
        frame_sizes[sample_index] = sonar_holder.cart_size();

        // adjust annotation to fit the preprocessed image
        std::vector<cv::Point> annotation_points = annotations[sample_index];
        cv::Mat annotation_mask;
        sonarlog_target_tracking::common::adjust_annotation(sonar_holder.cart_size(), annotation_points, annotation_points, annotation_mask);

        cv::Mat hog_input_image, hog_input_mask, hog_annotation_mask;
        double rotated_angle;
        hog_preprocessing(sonar_holder, annotation_points, annotation_mask,
                          hog_input_image, hog_input_mask, hog_annotation_mask,
                          rotated_angle, sample_index);

        cv::Size hog_win_size = cv::Size(208, 48);

        float scale_factor = 0.3;
        cv::Mat hog_input_image_reduced, hog_input_mask_reduced;
        cv::resize(hog_input_image, hog_input_image_reduced, cv::Size(0, 0), scale_factor, scale_factor);
        cv::resize(hog_input_mask, hog_input_mask_reduced, cv::Size(0, 0), scale_factor, scale_factor);

        // increase input mask
        image_util::dilate(hog_input_mask_reduced, hog_input_mask_reduced, cv::Size(25, 25), 2);

        // set region of interest
        cv::Rect bounding_rect = image_util::get_bounding_rect(hog_input_mask_reduced);

        // adjust the bounding_rect size
        if (bounding_rect.width < hog_win_size.width) bounding_rect.width = hog_win_size.width;
        if (bounding_rect.height < hog_win_size.height) bounding_rect.height = hog_win_size.height;

        cv::Mat input_image;
        hog_input_image_reduced(bounding_rect).convertTo(input_image, CV_8U, 255.0);

        // detect the target using multi scale slide window
        std::vector<cv::Rect> locations;
        std::vector<double> found_weights;
        hog.detectMultiScale(input_image, locations, found_weights, 0, cv::Size(8, 8), cv::Size(0, 0), 1.125, 1);

        // build locations masks
        std::vector<cv::Mat> loc_masks;
        std::vector<cv::RotatedRect> loc_boxs;
        build_locations_masks(locations, hog_input_image.size(), sonar_holder.cart_size(), bounding_rect.tl(),
                              scale_factor, rotated_angle, loc_masks, loc_boxs);

        // create annotation mask
        cv::Mat annotation_box_mask = cv::Mat::zeros(sonar_holder.cart_size(), CV_8UC1);
        image_util::create_min_area_rect_mask(annotation_points, annotation_box_mask);

        double annotation_area = cv::sum(annotation_box_mask)[0]/255.0;

        std::vector<cv::Mat>::const_iterator it;

        std::stringstream ss;
        for (it=loc_masks.begin(); it != loc_masks.end(); it++) {
            cv::Mat intersection_mask;
            cv::bitwise_and(annotation_box_mask, *it, intersection_mask);
            double location_area = cv::sum(*it)[0]/255.0;
            double intersection_area = cv::sum(intersection_mask)[0]/255.0;
            double intersection_percentage = intersection_area/(annotation_area+location_area-intersection_area);

            accuracy_levels[sample_index].push_back(intersection_percentage);
            classifier_weights[sample_index].push_back(found_weights[it-loc_masks.begin()]);
            detector_results[sample_index].push_back(loc_boxs[it-loc_masks.begin()]);

            ss << " ";
            ss << intersection_percentage;
        }

        // draw annotation
        cv::Mat annotation_image;
        image_util::draw_contour_min_area_rect(sonar_holder.cart_image(), annotation_image, annotation_points);
        image_util::draw_text(annotation_image, "Original", cv::Point(5, annotation_image.rows), cv::Scalar(0, 0, 255));

        // show detector locations
        cv::Mat loc_result_image;
        draw_locations(sonar_holder.cart_image(), loc_result_image, loc_boxs);
        image_util::draw_text(loc_result_image, "Result", cv::Point(5, loc_result_image.rows), cv::Scalar(0, 0, 255));
        image_util::draw_text(loc_result_image, ss.str(), cv::Point(170, loc_result_image.rows), cv::Scalar(0, 0, 255));

        cv::Mat out;
        cv::hconcat(annotation_image, loc_result_image, out);
        image_util::show_image("result", out, 2);
        cv::waitKey((sample_index==start_sample) ? -1 : 25);
    }
    printf("\n");
}

bool file_exists(std::string filename) {

    if (!boost::filesystem::exists(filename) &&
        !boost::filesystem::exists(boost::filesystem::path(boost::filesystem::current_path()).string() + "/" + filename)) {
        return false;
    }

    return true;
}


int main(int argc, char const *argv[]) {

    const std::string logfiles[] = {
        "/home/gustavoneves/masters_degree/dataset/logs/20161206-1642_001196-002416_gemini.0.log"
    };

    const std::string logannotation_files[] = {
        "/home/gustavoneves/masters_degree/dataset/logs/20161206-1642_001196-002416_gemini.0_annotation.yml"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    for (uint32_t i = 0; i < sz; i++) {
        boost::filesystem::path logpath(logfiles[i]);
        std::cout << "parent_path: " << logpath.parent_path() << std::endl;
        std::cout << "stem: " << logpath.stem() << std::endl;

        std::string parent_path = logpath.parent_path().string();
        std::string log_name = logpath.stem().string();

        g_log_file_path = boost::filesystem::path(parent_path+"/"+log_name);

        if (!boost::filesystem::exists(g_log_file_path)){
            boost::filesystem::create_directory(g_log_file_path);
        }

        //load log samples
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        //load sonar samples
        std::vector<base::samples::Sonar> samples;
        sonarlog_target_tracking::common::load_samples(stream, samples);

        //load log annotations
        std::vector<std::vector<cv::Point> > annotations;
        sonarlog_target_tracking::common::load_log_annotation(logannotation_files[i], "jequitaia", annotations);

        if (!file_exists("jequitaia_training.yml")) {

            //load training data
            std::vector<cv::Mat> gradient_positive, gradient_negative;

            for (int i = 0; i < samples.size(); i+=50) {
                load_training_data(samples, annotations, i, 20, gradient_positive, gradient_negative);
            }

            // prepare training data
            std::vector<int> labels;
            cv::Mat train_data;
            prepare_training_data(gradient_positive, gradient_negative, labels, train_data);

            // perform the training
            perform_svm_train(labels, train_data, "jequitaia_training.yml");
        }

        std::vector<std::vector<double> > accuracy_levels;
        std::vector<std::vector<double> > classifier_weights;
        std::vector<std::vector<cv::RotatedRect> > detector_results;
        std::vector<cv::Size> frame_sizes;

        // perform the testing
        perform_svm_test(samples, annotations, "jequitaia_training.yml", accuracy_levels,
            classifier_weights, detector_results, frame_sizes);

        // save detection result
        sonarlog_target_tracking::common::save_detection_results(log_name+std::string("_detection_result.yml"),
            accuracy_levels, classifier_weights, detector_results, frame_sizes, annotations);
    }
}
