#ifndef sonarlog_target_tracking_DetectionResult_hpp
#define sonarlog_target_tracking_DetectionResult_hpp

#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <stdint.h>


namespace sonarlog_target_tracking {

struct DetectionResultItem {

    DetectionResultItem() {
        type = 0;
        result = 0;
        overlap = 0;
        weight = 0;
        name = "";
    }

    DetectionResultItem(
        int type,
        int result,
        double overlap,
        double weight,
        const std::string& name)
        : type(type)
        , result(result)
        , overlap(overlap)
        , weight(weight)
        , name(name)
    {
    }

    int type;
    int result;
    double overlap;
    double weight;
    std::string name;
};

class DetectionResult {

public:

    void add(int type, int result, double overlap, double weight, const std::string& name) {
        add(DetectionResultItem(type, result, overlap, weight, name));
    }

    void add(DetectionResultItem item) {
        items_.push_back(item);
    }

    void reset() {
        items_.clear();
    }

    void csv_write(const std::string& filename) {
        std::stringstream ss;

        ss << "name,type,result,overlap,weight\n";
        for (size_t i = 0; i < items_.size(); i++) {
            ss << items_[i].name;
            ss << ",";
            ss << to_string(items_[i].type);
            ss << ",";
            ss << to_string(items_[i].result);
            ss << ",";
            ss << to_string(items_[i].overlap);
            ss << ",";
            ss << to_string(items_[i].weight);
            ss << "\n";
        }

        std::ofstream out;
        out.open(filename.c_str());
        out << ss.str();
        out.close();
    }

    std::string to_string(double val) {
        char buffer[100];
        snprintf(buffer, 100, "%.5f", val);
        return std::string(buffer);
    }

    std::string to_string(int  val) {
        char buffer[100];
        snprintf(buffer, 100, "%d", val);
        return std::string(buffer);
    }


private:
    std::vector<DetectionResultItem> items_;

};

}

#endif
