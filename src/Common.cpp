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


} /* namespace common */
    
} /* namespace sonarlog_target_tracking */
