#ifndef DetectionStats_hpp
#define DetectionStats_hpp

#include <iostream>
#include <map>
#include <vector>
#include <opencv2/ml/ml.hpp>
#include <yaml-cpp/yaml.h>

namespace sonarlog_target_tracking {

class DetectionStats {

public:
    DetectionStats() {}

    DetectionStats(const std::string& filename) {
        LoadFromFile(filename);    
    }

    DetectionStats(const std::vector<std::vector<double> >& accuracy_levels, 
                   const std::vector<std::vector<double> >& classifier_weights,
                   const std::vector<std::vector<cv::RotatedRect> >& detection_results,
                   const std::vector<cv::Size>& frame_sizes,
                   const std::vector<std::vector<cv::Point> >& annotations);

    void Save(const std::string& filename) const;

private:

    void LoadFromFile(const std::string& filename);

    std::vector<std::map<std::string, float> > RotatedRectVectorToMapVector(const std::vector<cv::RotatedRect>& rrects) const;

    std::map<std::string, float> RotatedRectToMap(const cv::RotatedRect& rrect) const;
    
    std::map<std::string, int> SizeToMap(const cv::Size size) const;
    
    std::map<std::string, int> PointToMap(const cv::Point& point) const;

    std::vector<std::map<std::string, int> > PointVectorToMapVector(const std::vector<cv::Point>& points) const;

    std::vector<std::vector<double> > accuracy_levels_;
    std::vector<std::vector<double> > classifier_weights_;
    std::vector<std::vector<cv::RotatedRect> > detection_results_;
    std::vector<cv::Size > frame_sizes_;
    std::vector<std::vector<cv::Point> > annotations_;

};

} /* namespace sonarlog_target_tracking */


#endif /* DetectionStats_hpp */
