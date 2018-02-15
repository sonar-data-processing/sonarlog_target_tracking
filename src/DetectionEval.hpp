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

    const cv::Mat& overlap_region_image() const {
        return overlap_region_image_;
    }

    double true_positive_rate() const {
        return true_positive_rate_;
    }

    double recall() const {
        return true_positive_rate_;
    }

    double false_positive_rate() const {
        return false_positive_rate_;
    }

    double fall_out() const {
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

    double precision() const {
        return precision_;
    }

    double ground_truth_area() const {
        return ground_truth_area_;
    }

    double detected_area() const {
        return detected_area_;
    }

    double overlap_region() const {
        return overlap_region_;
    }

    double f1_score() const {
        return f1_score_;
    }

    const cv::Point2f& ground_truth_position() const {
        return ground_truth_position_;
    }

    const cv::Point2f& detected_position() const {
        return detected_position_;
    }

protected:

    void EvalCalculate();

    void CalculateStats(
        const cv::Mat& ground_truth_mask,
        const cv::Mat& ground_truth_mask_inv,
        const cv::Mat& detection_mask,
        const cv::Mat& detection_mask_inv);

    void CalculatePositions(
        const cv::Mat& ground_truth_mask,
        const cv::Mat& detection_mask);


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
    double precision_;
    double ground_truth_area_;
    double detected_area_;
    double overlap_region_;
    double f1_score_;

    cv::Point2f detected_position_;
    cv::Point2f ground_truth_position_;

    cv::Mat overlap_region_image_;
};

struct DetectionEvalList {

    enum Columns {
        TRUE_POSITIVE = 0,
        FALSE_POSITIVE,
        TRUE_NEGATIVE,
        FALSE_NEGATIVE,
        TRUE_POSITIVE_RATE,
        FALSE_POSITIVE_RATE,
        ACCURACY,
        PRECISION,
        OVERLAP_REGION,
        F1_SCORE,
        GROUND_TRUTH_AREA,
        GROUND_TRUTH_POS_X,
        GROUND_TRUTH_POS_Y,
        DETECTED_AREA,
        DETECTED_POS_X,
        DETECTED_POS_Y,
        COLUMNS_END
    };

    void add_detection_evaluation(const DetectionEval& detection_eval) {
        data_eval[TRUE_POSITIVE].push_back(detection_eval.true_positive());
        data_eval[FALSE_POSITIVE].push_back(detection_eval.false_positive());
        data_eval[TRUE_NEGATIVE].push_back(detection_eval.true_negative());
        data_eval[FALSE_NEGATIVE].push_back(detection_eval.false_negative());
        data_eval[TRUE_POSITIVE_RATE].push_back(detection_eval.true_positive_rate());
        data_eval[FALSE_POSITIVE_RATE].push_back(detection_eval.false_positive_rate());
        data_eval[ACCURACY].push_back(detection_eval.accuracy());
        data_eval[PRECISION].push_back(detection_eval.precision());
        data_eval[OVERLAP_REGION].push_back(detection_eval.overlap_region());
        data_eval[F1_SCORE].push_back(detection_eval.f1_score());
        data_eval[GROUND_TRUTH_AREA].push_back(detection_eval.ground_truth_area());
        data_eval[GROUND_TRUTH_POS_X].push_back(detection_eval.ground_truth_position().x);
        data_eval[GROUND_TRUTH_POS_Y].push_back(detection_eval.ground_truth_position().y);
        data_eval[DETECTED_AREA].push_back(detection_eval.detected_area());
        data_eval[DETECTED_POS_X].push_back(detection_eval.detected_position().x);
        data_eval[DETECTED_POS_Y].push_back(detection_eval.detected_position().y);
    }

    void csv_write(const std::string& filename) {
        std::stringstream ss;

        for (size_t c = 0; c < COLUMNS_END; c++) {
            ss << column_name(static_cast<Columns>(c));
            if (c < COLUMNS_END-1) ss << ","; else ss << "\n";
        }

        for (size_t i=0; i<data_eval[TRUE_POSITIVE].size(); i++) stream_write(ss, i);

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

            assert(cols.size() == COLUMNS_END);
            for (size_t i = 0; i < COLUMNS_END; i++) data_eval[i].push_back(cols[i]);
        }
    }

    void stream_write(std::stringstream& ss, int sample_index) {
        for (size_t i = 0; i < COLUMNS_END; i++) {
            char buffer[100];
            snprintf(buffer, 100, "%.5f", data_eval[i][sample_index]);
            ss << std::string(buffer);
            if (i < COLUMNS_END-1) ss << ","; else ss << "\n";
        }
    }

    std::string column_name(Columns column){
        assert(column < COLUMNS_END);
        switch (column) {
            case TRUE_POSITIVE:
                return "TRUE_POSITIVE";
            case FALSE_POSITIVE:
                return "FALSE_POSITIVE";
            case TRUE_NEGATIVE:
                return "TRUE_NEGATIVE";
            case FALSE_NEGATIVE:
                return "FALSE_NEGATIVE";
            case TRUE_POSITIVE_RATE:
                return "TRUE_POSITIVE_RATE";
            case FALSE_POSITIVE_RATE:
                return "FALSE_POSITIVE_RATE";
            case ACCURACY:
                return "ACCURACY";
            case PRECISION:
                return "PRECISION";
            case OVERLAP_REGION:
                return "OVERLAP_REGION";
            case F1_SCORE:
                return "F1_SCORE";
            case GROUND_TRUTH_AREA:
                return "GROUND_TRUTH_AREA";
            case GROUND_TRUTH_POS_X:
                return "GROUND_TRUTH_POS_X";
            case GROUND_TRUTH_POS_Y:
                return "GROUND_TRUTH_POS_Y";
            case DETECTED_AREA:
                return "DETECTED_AREA";
            case DETECTED_POS_X:
                return "DETECTED_POS_X";
            case DETECTED_POS_Y:
                return "DETECTED_POS_Y";
        }
    }

    std::vector<double> data_eval[COLUMNS_END];
};

} // namespace sonarlog_target_tracking


#endif /* sonarlog_target_tracking_DetectionEval_hpp */
