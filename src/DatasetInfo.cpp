#include <iostream>
#include "DatasetInfo.hpp"

namespace sonarlog_target_tracking {

DatasetInfo::DatasetInfo()
{
}

DatasetInfo::DatasetInfo(const std::string& filename)
{
    Load(filename);
}

DatasetInfo::~DatasetInfo()
{
}

void DatasetInfo::Load(const std::string& filename)
{
    YAML::Node node = YAML::LoadFile(filename);
    assert(node.Type() == YAML::NodeType::Sequence);

    for (size_t i=0; i<node.size(); i++) {
        DatasetInfoEntry entry;
        NodeToEntry(node[i], entry);
        entries_.push_back(entry);
    }
}

void DatasetInfo::NodeToEntry(const YAML::Node& node, DatasetInfoEntry& entry) {
    assert(node["log-filename"]);
    assert(node["stream-name"]);
    entry.log_filename = node["log-filename"].as<std::string>();
    entry.stream_name = node["stream-name"].as<std::string>();
    entry.from_index = (node["from_index"]) ?  node["from_index"].as<int>() : -1;
    entry.to_index = (node["to_index"]) ?  node["to_index"].as<int>() : -1;
    entry.annotation_filename = (node["annotation-filename"]) ? node["annotation-filename"].as<std::string>() : "";
    entry.annotation_name = (node["annotation-name"]) ? node["annotation-name"].as<std::string>() : "";
}

} /* namespace sonarlog_target_tracking */
