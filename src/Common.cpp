#include <boost/filesystem.hpp>
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/Utils.hpp"
#include "sonarlog_annotation/AnnotationFileReader.hpp"
#include "Common.hpp"

using namespace sonar_processing;
using namespace sonarlog_annotation;

namespace sonarlog_target_tracking {

namespace common {

void load_log_annotation(const std::string& logannotation_file, const std::string& annotation_name, std::vector<std::vector<cv::Point> >& annotation_points) {
    AnnotationFileReader annotation_file_reader(logannotation_file);
    std::vector<AnnotationFileReader::AnnotationMap> annotations = annotation_file_reader.read();

    if (annotations.empty()) {
        return;
    }

    annotation_points.resize(annotations.size());

    for (size_t i = 0; i < annotations.size(); i++) {
        AnnotationFileReader::AnnotationMap::iterator it;
        it = annotations[i].find(annotation_name);

        if (it != annotations[i].end()) {
            utils::point2f_to_point2i(it->second, annotation_points[i]);
        }
    }
}

void load_samples(rock_util::LogStream& stream, std::vector<base::samples::Sonar>& samples, size_t total_samples) {
    printf("Loading log samples\n\033[s");
    base::samples::Sonar sample;

    total_samples = (total_samples==-1) ? stream.total_samples() : total_samples;

    do {
        printf("\033[uLoading sample: %ld of %ld", stream.current_sample_index()+1, total_samples);
        fflush(stdout);
        //get sample
        stream.next<base::samples::Sonar>(sample);
        // add sample to vector
        samples.push_back(sample);
        // got to next sample
    } while(stream.current_sample_index() < total_samples);
    printf("\n");
}

void adjust_annotation(cv::Size size, const std::vector<cv::Point>& src_points, std::vector<cv::Point>& dst_points, cv::OutputArray annotation_mask) {
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(src_points);
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), CV_FILLED);
    dst_points = preprocessing::find_biggest_contour(mask);
    mask.copyTo(annotation_mask);
}

bool file_exists(std::string filename) {
    if (!boost::filesystem::exists(filename) &&
        !boost::filesystem::exists(boost::filesystem::path(boost::filesystem::current_path()).string() + "/" + filename)){
        return false;
    }
    return true;
}

void exec_samples(rock_util::LogStream& stream, int first_sample, int last_sample, void (*exec_samples_callback)(const base::samples::Sonar& sample, void *user_data), void *user_data) {
    stream.set_current_sample_index(first_sample);
    do {
        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);
        exec_samples_callback(sample, user_data);
    } while(stream.current_sample_index() < last_sample);
}

void exec_samples_from_dataset_entry(const DatasetInfoEntry& dataset_entry, void (*exec_samples_callback)(const base::samples::Sonar& sample, void *user_data), void *user_data) {
    rock_util::LogReader reader(dataset_entry.log_filename);
    std::cout << dataset_entry.to_string() << std::endl;

    rock_util::LogStream stream = reader.stream(dataset_entry.stream_name);

    std::vector<std::vector<cv::Point> > annotations;
    if (!dataset_entry.annotation_filename.empty() &&
        !dataset_entry.annotation_name.empty()) {
        load_log_annotation(dataset_entry.annotation_filename, dataset_entry.annotation_name, annotations);
        std::cout << "Annotation openned with success" << std::endl;
    }

    int first_sample = (dataset_entry.from_index >= 1 && dataset_entry.from_index <= stream.total_samples()) ? dataset_entry.from_index-1 : 0;
    int last_sample = (dataset_entry.to_index >= 1 && dataset_entry.to_index <= stream.total_samples()) ? dataset_entry.to_index-1 : stream.total_samples()-1;

    exec_samples(stream, first_sample, last_sample, exec_samples_callback, user_data);
}

} /* namespace common */

} /* namespace sonarlog_target_tracking */
