#ifndef sonarlog_target_tracking_DatasetInfo_hpp
#define sonarlog_target_tracking_DatasetInfo_hpp

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace sonarlog_target_tracking {

struct DatasetInfoEntry {
    std::string log_filename;
    std::string stream_name;
    std::string annotation_filename;
    std::string annotation_name;
    int from_index;
    int to_index;

    std::string to_string() const {
        std::stringstream ss;
        ss << "log_filename: " << log_filename << "\n";
        ss << "stream_name: " << stream_name << "\n";
        ss << "from_index: " << from_index << "\n";
        ss << "to_index: " << to_index << "\n";
        ss << "annotation_filename: " << ((annotation_filename.empty()) ? "empty" : annotation_filename) << "\n";
        ss << "annotation_name: " << ((annotation_name.empty()) ? "empty" : annotation_name) << "\n";
        return ss.str();
    }
};

class DatasetInfo {

public:

    DatasetInfo();

    DatasetInfo(const std::string& filename);

    ~DatasetInfo();

    // load the dataset info YML file
    void Load(const std::string& filename);

    // returns dataset info entries
    const std::vector<DatasetInfoEntry>& entries() const {
        return entries_;
    }

private:

    void NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry);

    std::vector<DatasetInfoEntry> entries_;

};

} /* namespace sonarlog_target_tracking */

#endif
