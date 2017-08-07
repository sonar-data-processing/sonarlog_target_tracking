#include <fstream>
#include "DetectionStats.hpp"

namespace sonarlog_target_tracking {

DetectionStats::DetectionStats(
    const std::vector<std::vector<double> >& accuracy_levels,
    const std::vector<std::vector<double> >& classifier_weights,
    const std::vector<std::vector<cv::RotatedRect> >& detection_results,
    const std::vector<cv::Size>& frame_sizes,
    const std::vector<std::vector<cv::Point> >& annotations)
    : accuracy_levels_(accuracy_levels)
    , classifier_weights_(classifier_weights)
    , detection_results_(detection_results)
    , frame_sizes_(frame_sizes)
    , annotations_(annotations)
{
}

void DetectionStats::Save(const std::string& filename) const {
    assert(accuracy_levels_.size() == classifier_weights_.size());
    assert(accuracy_levels_.size() == detection_results_.size());

    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (size_t i = 0; i < accuracy_levels_.size(); i++) {
        out << YAML::BeginMap;
        out << YAML::Key << "accuracy_levels" << YAML::Flow << accuracy_levels_[i];
        out << YAML::Key << "classifier_weights" << YAML::Flow << classifier_weights_[i];
        out << YAML::Key << "frame_size" << YAML::Flow << SizeToMap(frame_sizes_[i]);
        out << YAML::Key << "annotations" << YAML::Flow << PointVectorToMapVector(annotations_[i]);
        out << YAML::Key << "detection_results" << YAML::Flow << RotatedRectVectorToMapVector(detection_results_[i]);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    std::ofstream fout(filename.c_str());
    fout << out.c_str();
    fout.close();
}

void DetectionStats::LoadFromFile(const std::string& filename) {
    YAML::Node node = YAML::LoadFile(filename);
    assert(node.Type() == YAML::NodeType::Sequence);

    accuracy_levels_.resize(node.size());
    classifier_weights_.resize(node.size());
    detection_results_.resize(node.size());
    annotations_.resize(node.size());
    frame_sizes_.resize(node.size());

    for (size_t i=0;i<node.size();i++) {
        assert(node[i].Type() == YAML::NodeType::Map);

        // Get accuracy level
        node2vector(node[i]["accuracy_levels"], accuracy_levels_[i]);

        // Get classifier weights
        node2vector(node[i]["classifier_weights"], classifier_weights_[i]);

        // Get detection results
        NodeToRotatedRectVector(node[i]["detection_results"], detection_results_[i]);

        // Get annotations
        NodeToPointVector(node[i]["annotations"], annotations_[i]);

        // Get frame size
        NodeToSize(node[i]["frame_size"], frame_sizes_[i]);
    }
}

std::map<std::string, float> DetectionStats::RotatedRectToMap(const cv::RotatedRect& rrect) const {
    std::map<std::string, float> map;
    map.insert(std::make_pair("cx", rrect.center.x));
    map.insert(std::make_pair("cy", rrect.center.y));
    map.insert(std::make_pair("width", rrect.size.width));
    map.insert(std::make_pair("height", rrect.size.height));
    map.insert(std::make_pair("angle", rrect.angle));
    return map;
}

std::map<std::string, int> DetectionStats::SizeToMap(const cv::Size size) const {
    std::map<std::string, int> map;
    map.insert(std::make_pair("width", size.width));
    map.insert(std::make_pair("height", size.height));
    return map;
}

std::vector<std::map<std::string, float> > DetectionStats::RotatedRectVectorToMapVector(const std::vector<cv::RotatedRect>& rrects) const {
    std::vector<std::map<std::string, float> > vec(rrects.size());
    for (std::vector<cv::RotatedRect>::const_iterator it = rrects.begin(); it != rrects.end(); it++) {
        vec[it-rrects.begin()] = RotatedRectToMap(*it);
    }
    return vec;
}

std::map<std::string, int> DetectionStats::PointToMap(const cv::Point& point) const {
    std::map<std::string, int> map;
    map.insert(std::make_pair("x", point.x));
    map.insert(std::make_pair("y", point.y));
    return map;
}

std::vector<std::map<std::string, int> > DetectionStats::PointVectorToMapVector(const std::vector<cv::Point>& points) const {
    std::vector<std::map<std::string, int> > vec(points.size());
    for (std::vector<cv::Point>::const_iterator it = points.begin(); it != points.end(); it++) {
        vec[it-points.begin()] = PointToMap(*it);
    }
    return vec;
}

void DetectionStats::NodeToRotatedRectVector(const YAML::Node& node, std::vector<cv::RotatedRect>& vec) {
    assert(node != NULL);
    assert(node.IsSequence());

    if (node.size()==0) return;

    vec.resize(node.size());
    for (size_t i=0; i<node.size(); i++) {
        NodeToRotatedRect(node[i], vec[i]);
    }
}

void DetectionStats::NodeToRotatedRect(const YAML::Node& node, cv::RotatedRect& rrect) {
    assert(node != NULL);
    assert(node.IsMap());
    assert(node["angle"]);
    assert(node["cx"] && node["cy"]);
    assert(node["width"] && node["height"]);
    rrect= cv::RotatedRect(cv::Point2f(node["cx"].as<float>(), node["cy"].as<float>()),
                           cv::Size2f(node["width"].as<float>(), node["height"].as<float>()),
                           node["angle"].as<float>());
}

void DetectionStats::NodeToPointVector(const YAML::Node& node, std::vector<cv::Point>& vec) {
    assert(node != NULL);
    assert(node.IsSequence());

    if (node.size()==0) return;

    vec.resize(node.size());

    for (size_t i=0; i<node.size(); i++) {
        NodeToPoint(node[i], vec[i]);
    }
}

void DetectionStats::NodeToPoint(const YAML::Node& node, cv::Point& point) {
    assert(node != NULL);
    assert(node.IsMap());
    assert(node["x"] && node["y"]);
    point = cv::Point(node["x"].as<int>(), node["y"].as<int>());
}

void DetectionStats::NodeToSize(const YAML::Node& node, cv::Size& size) {
    assert(node != NULL);
    assert(node.IsMap());
    assert(node["width"] && node["height"]);
    size = cv::Point(node["width"].as<int>(), node["height"].as<int>());
}


} /* namespace sonarlog_target_tracking */
