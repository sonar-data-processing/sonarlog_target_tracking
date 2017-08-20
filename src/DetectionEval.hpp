#ifndef sonarlog_target_tracking_DetectionEval_hpp
#define sonarlog_target_tracking_DetectionEval_hpp

#include <fstream>
#include <vector>
#include <cstdlib>
#include <opencv2/opencv.hpp>

namespace sonarlog_target_tracking
{

class DetectionEval
{
public:
    DetectionEval(
        const std::vector<cv::RotatedRect>& locations,
        const std::vector<cv::Point>& annotations,
        cv::Size frame_size);

    ~DetectionEval();

    const cv::Mat& intersection_area_image() const {
        return intersection_area_image_;
    }

    double true_positive_rate() const {
        return true_positive_rate_;
    }

    double false_positive_rate() const {
        return false_positive_rate_;
    }

    double true_positive() const {
        return true_positive_;
    }

    double false_positive() const {
        return false_positive_;
    }

    double true_negative() const {
        return true_negative_;
    }

    double false_negative() const {
        return false_negative_;
    }

    double accuracy() const {
        return accuracy_;
    }

protected:

    void EvalCalculate();

    void CalculateIntersectionArea(
        const cv::Mat& ground_truth_mask,
        const cv::Mat& ground_truth_mask_inv,
        const cv::Mat& detection_mask,
        const cv::Mat& detection_mask_inv);

    std::vector<cv::RotatedRect> locations_;
    std::vector<cv::Point> annotations_;
    cv::Size frame_size_;

    double true_positive_;
    double false_positive_;
    double true_negative_;
    double false_negative_;
    double true_positive_rate_;
    double false_positive_rate_;
    double accuracy_;

    cv::Mat intersection_area_image_;
};

struct DetectionEvalList {

    void add_detection_evaluation(const DetectionEval& detection_eval) {
        true_positive_rate_list.push_back(detection_eval.true_positive_rate());
        false_positive_rate_list.push_back(detection_eval.false_positive_rate());
        true_positive_list.push_back(detection_eval.true_positive());
        false_positive_list.push_back(detection_eval.false_positive());
        true_negative_list.push_back(detection_eval.true_negative());
        false_negative_list.push_back(detection_eval.false_negative());
        accuracy_list.push_back(detection_eval.accuracy());
    }

    void csv_write(const std::string& filename) {
        std::stringstream ss;

        for (size_t i=0; i<accuracy_list.size(); i++)  {
            stream_write(
                ss,
                true_positive_list[i],
                false_positive_list[i],
                true_negative_list[i],
                false_negative_list[i],
                true_positive_rate_list[i],
                false_positive_rate_list[i],
                accuracy_list[i]);
        }

        std::ofstream out;
        out.open(filename.c_str());
        out << ss.str();
        out.close();
    }

    void csv_read(const std::string& filename) {
        std::ifstream in(filename.c_str());
        std::string line;
        while (std::getline(in, line)) {
            std::vector<double> cols;
            std::istringstream ss(line);
            std::string token;
            while (std::getline(ss, token, ',')) cols.push_back(std::atof(token.c_str()));
            assert(cols.size() == 7);
            true_positive_list.push_back(cols[0]);
            false_positive_list.push_back(cols[1]);
            true_negative_list.push_back(cols[2]);
            false_negative_list.push_back(cols[3]);
            true_positive_rate_list.push_back(cols[4]);
            false_positive_rate_list.push_back(cols[5]);
            accuracy_list.push_back(cols[6]);
        }
    }

    void stream_write(
        std::stringstream& ss,
        double true_positive,
        double false_positive,
        double true_negative,
        double false_negative,
        double true_positive_rate,
        double false_positive_rate,
        double accuracy) {
        char buffer[1024];
        snprintf(buffer, 1024, "%.20f,%.20f,%.20f,%.20f,%.20f,%.20f,%.20f\n",
                true_positive,
                false_positive,
                true_negative,
                false_negative,
                true_positive_rate,
                false_positive_rate,
                accuracy);
        ss << std::string(buffer);
    }

    std::vector<double> true_positive_rate_list;
    std::vector<double> false_positive_rate_list;
    std::vector<double> true_positive_list;
    std::vector<double> false_positive_list;
    std::vector<double> true_negative_list;
    std::vector<double> false_negative_list;
    std::vector<double> accuracy_list;
};

} // namespace sonarlog_target_tracking


#endif /* sonarlog_target_tracking_DetectionEval_hpp */
