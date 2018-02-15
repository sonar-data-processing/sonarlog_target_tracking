#include <iostream>
#include "VideoConverterSettings.hpp"

namespace sonarlog_target_tracking {

VideoConverterSettings::VideoConverterSettings()
{
}

VideoConverterSettings::VideoConverterSettings(const std::string& filepath)
{
    Load(filepath);
}

VideoConverterSettings::~VideoConverterSettings()
{
}

void VideoConverterSettings::Load(const std::string& filepath)
{
    YAML::Node node = YAML::LoadFile(filepath);
    assert(node.Type() == YAML::NodeType::Map);

    // load video matrix settings
    LoadVideoMatrixSettings(node["video-matrix"]);
}

void VideoConverterSettings::LoadVideoMatrixSettings(const YAML::Node& node)
{
    if (!node) return;
    assert(node["rows"]);
    assert(node["cols"]);
    assert(node["output-file"]);
    assert(node["image-size"]);
    assert(node["image-size"]["width"]);
    assert(node["image-size"]["height"]);

    video_matrix_settings_.cols  = node["cols"].as<int>();
    video_matrix_settings_.rows  = node["rows"].as<int>();
    video_matrix_settings_.image_width = node["image-size"]["width"].as<int>();
    video_matrix_settings_.image_height = node["image-size"]["height"].as<int>();
    video_matrix_settings_.output_file = node["output-file"].as<std::string>();

    assert(video_matrix_settings_.cols > 0);
    assert(video_matrix_settings_.rows > 0);

    LoadMatrixData(node["data"]);
}

void VideoConverterSettings::LoadMatrixData(const YAML::Node& node)
{
    assert(node);
    assert(node.Type() == YAML::NodeType::Sequence);

    int numel = video_matrix_settings_.cols * video_matrix_settings_.rows;
    assert(node.size() <= numel);

    video_matrix_settings_.data.resize(numel);

    for (size_t i = 0; i < node.size(); i++) {
        assert(node[i]["cell"]);
        assert(node[i]["cell"].Type() == YAML::NodeType::Sequence);
        std::vector<std::string> file_list;
        for (size_t j = 0; j < node[i]["cell"].size(); j++) {
            assert(node[i]["cell"][j]["filename"]);
            assert(node[i]["cell"][j]["stream-name"]);
            MatrixCellItem item;
            item.filename = node[i]["cell"][j]["filename"].as<std::string>();
            item.stream_name = node[i]["cell"][j]["stream-name"].as<std::string>();
            video_matrix_settings_.data[i].push_back(item);
        }
    }
}

} /* namespace sonarlog_target_tracking */
