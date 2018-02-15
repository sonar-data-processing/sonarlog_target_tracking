#ifndef sonarlog_target_tracking_VideoConverterSettings_hpp
#define sonarlog_target_tracking_VideoConverterSettings_hpp

#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace sonarlog_target_tracking {

struct MatrixCellItem {
    std::string filename;
    std::string stream_name;
};

struct VideoMatrixSettings
{
    std::string to_string() const {
        std::stringstream ss;
        ss << "cols: " << cols << "\n";
        ss << "rows: " << rows << "\n";
        ss << "image_width: " << image_width << "\n";
        ss << "image_height: " << image_height << "\n";
        ss << "data: " << "\n";
        ss <<  data_to_string() << "\n";
        return ss.str();
    }

    std::string data_to_string() const {
        std::stringstream ss;
        for (size_t i = 0; i < data.size(); i++) {
            ss << "- cell: " << i << "\n";
            if (data[i].empty()) {
                ss << "  - (empty)\n";
            }
            else {
                for (size_t j = 0; j < data[i].size(); j++) {
                    ss << "  - filename: " << data[i][j].filename << "\n";
                    ss << "    stream-name: " << data[i][j].stream_name << "\n";
                }
            }
        }
        return ss.str();
    }

    int cols;
    int rows;
    int image_width;
    int image_height;
    std::string output_file;
    std::vector<std::vector<MatrixCellItem> > data;
};

class VideoConverterSettings {

public:
    VideoConverterSettings();
    VideoConverterSettings(const std::string& filepath);

    virtual ~VideoConverterSettings();

    // load the dataset info YML file
    void Load(const std::string& filepath);

    VideoMatrixSettings video_matrix_settings() const {
        return video_matrix_settings_;
    }

private:

    // load video matrix settings
    void LoadVideoMatrixSettings(const YAML::Node& node);

    // load matrix data
    void LoadMatrixData(const YAML::Node& node);

    VideoMatrixSettings video_matrix_settings_;

};

} /* namespace sonarlog_target_tracking */

#endif /* sonarlog_target_tracking_VideoConverterSettings_hpp */
