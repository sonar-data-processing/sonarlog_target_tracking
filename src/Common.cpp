#include <boost/filesystem.hpp>
#include "sonar_processing/Preprocessing.hpp"
#include "sonar_processing/Utils.hpp"
#include "sonarlog_annotation/AnnotationFileReader.hpp"
#include "Common.hpp"

using namespace sonar_processing;
using namespace sonarlog_annotation;

namespace sonarlog_target_tracking {

namespace common {

void clip_interval(int& from, int& to, int length) {
    from = (from >= 1 && from <= length) ? from-1 : 0;
    to = (to >= 1 && to <= length) ? to-1 : length-1;
}

void load_log_annotation(const std::string& logannotation_file, const std::string& annotation_name, std::vector<std::vector<cv::Point> >& annotation_points) {

    if (!file_exists(logannotation_file)) {
        std::cout << "The log annotation file: " << logannotation_file << " does not exist." << std::endl;
        return;
    }

    AnnotationFileReader annotation_file_reader(logannotation_file);
    std::vector<AnnotationFileReader::AnnotationMap> annotations = annotation_file_reader.read();

    if (annotations.empty()) {
        annotation_points.clear();
        return;
    }

    annotation_points.clear();
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

void load_samples_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    std::vector<base::samples::Sonar>& result_samples,
    std::vector<std::vector<cv::Point> >& result_annotations)
{
    rock_util::LogReader reader(dataset_entry.log_filename);
    rock_util::LogStream stream = reader.stream(dataset_entry.stream_name);

    std::vector<std::vector<cv::Point> > annotations;
    load_log_annotation(dataset_entry.annotation_filename, dataset_entry.annotation_name, annotations);

    int first_sample = dataset_entry.from_index;
    int last_sample = dataset_entry.to_index;
    clip_interval(first_sample, last_sample, stream.total_samples());

    stream.set_current_sample_index(first_sample);
    int sample_count = 0;
    base::samples::Sonar sample;

    printf("Loading log samples\n\033[s");
    do {
        printf("\033[uLoading sample: %ld", stream.current_sample_index()+1);
        fflush(stdout);

        std::vector<cv::Point> annotation_points = annotations[stream.current_sample_index()];
        result_annotations.push_back(annotation_points);
        stream.next<base::samples::Sonar>(sample);
        result_samples.push_back(sample);
        sample_count++;
    } while(stream.current_sample_index() <= last_sample);
    printf("\nTotal samples: %d\n", sample_count);
}
void adjust_annotation(cv::Size size, const std::vector<cv::Point>& src_points, std::vector<cv::Point>& dst_points, cv::Mat& annotation_mask) {
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(src_points);
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    cv::drawContours(mask, contours, -1, cv::Scalar(255), CV_FILLED);
    dst_points = preprocessing::find_biggest_contour(mask);
    mask.copyTo(annotation_mask);
}

bool file_exists(std::string filename) {
    if (filename.empty() ||
        !boost::filesystem::exists(filename) &&
        !boost::filesystem::exists(boost::filesystem::path(boost::filesystem::current_path()).string() + "/" + filename)){
        return false;
    }
    return true;
}

void exec_samples(
    rock_util::LogStream& stream,
    int first_sample,
    int last_sample,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data)
{
    stream.set_current_sample_index(first_sample);
    do {
        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);
        exec_samples_callback(sample, stream.current_sample_index()-1, user_data);
    } while(stream.current_sample_index() <= last_sample);
}

void exec_samples_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data)
{
    rock_util::LogReader reader(dataset_entry.log_filename);
    rock_util::LogStream stream = reader.stream(dataset_entry.stream_name);

    int first_sample = dataset_entry.from_index;
    int last_sample = dataset_entry.to_index;
    clip_interval(first_sample, last_sample, stream.total_samples());
    exec_samples(stream, first_sample, last_sample, exec_samples_callback, user_data);
}

void exec_training_samples_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    EXEC_SAMPLE_CALLBACK exec_samples_callback,
    void *user_data)
{
    if (!dataset_entry.training_intervals.empty()) {
        rock_util::LogReader reader(dataset_entry.log_filename);
        rock_util::LogStream stream = reader.stream(dataset_entry.stream_name);

        std::vector<SampleInterval>::const_iterator it = dataset_entry.training_intervals.begin();
        for (it; it != dataset_entry.training_intervals.end(); it++) {
            int first_sample = (*it).from;
            int last_sample = (*it).to;
            clip_interval(first_sample, last_sample, stream.total_samples());
            exec_samples(stream, first_sample, last_sample, exec_samples_callback, user_data);
        }
    }

}

void load_training_data_from_dataset_entry(
    const DatasetInfoEntry& dataset_entry,
    std::vector<base::samples::Sonar>& training_samples,
    std::vector<std::vector<cv::Point> >& training_annotations)
{
    if (dataset_entry.training_intervals.empty()) {
        printf("There is no training interval for this log entry.\n");
        return;
    }

    if (dataset_entry.annotation_filename.empty()) {
        std::cout << "There is no annotation filename for " << dataset_entry.log_filename << " entry." << std::endl;;
    }

    if (dataset_entry.annotation_name.empty()) {
        std::cout << "There is no annotation name for " << dataset_entry.log_filename << " entry." << std::endl;
    }


    rock_util::LogReader reader(dataset_entry.log_filename);
    rock_util::LogStream stream = reader.stream(dataset_entry.stream_name);

    std::vector<std::vector<cv::Point> > annotations;
    load_log_annotation(dataset_entry.annotation_filename, dataset_entry.annotation_name, annotations);

    std::vector<SampleInterval>::const_iterator it = dataset_entry.training_intervals.begin();
    printf("Loading training log samples\n\033[s");

    int samples_count = 0;
    for (it; it != dataset_entry.training_intervals.end(); it++) {
        int first_sample = (*it).from;
        int last_sample = (*it).to;
        clip_interval(first_sample, last_sample, stream.total_samples());
        base::samples::Sonar sample;
        stream.set_current_sample_index(first_sample);
        do {
            printf("\033[uLoading sample: %ld", stream.current_sample_index()+1);
            fflush(stdout);

            std::vector<cv::Point> annotation_points;
            if (!annotations.empty()) {
                annotation_points = annotations[stream.current_sample_index()];
            }

            training_annotations.push_back(annotation_points);

            base::samples::Sonar sample;
            stream.next<base::samples::Sonar>(sample);
            training_samples.push_back(sample);
            samples_count++;
        } while(stream.current_sample_index() <= last_sample);
    }
    printf("\nTotal training log samples: %d\n", samples_count);
}


} /* namespace common */

} /* namespace sonarlog_target_tracking */
