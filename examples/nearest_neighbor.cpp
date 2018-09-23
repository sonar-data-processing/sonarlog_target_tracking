
#include <base/samples/Sonar.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include "SonarView.hpp"
#include "Common.hpp"
#include "ArgumentParser.hpp"
#include "DatasetInfo.hpp"

using namespace sonar_processing;
using namespace sonarlog_target_tracking;


// receive samples from sonar log reader
void sample_receiver_callback(const base::samples::Sonar &sample, int sample_index, void *user_data)
{
    std::cout << "sample_index:" <<  sample_index << std::endl;
    SonarHolder *holder = reinterpret_cast<SonarHolder *>(user_data);
    common::load_sonar_holder(sample, *holder);

    std::cout << "beam_width: " << holder->beam_width() << std::endl;
    std::cout << "beam_count: " << holder->beam_count() << std::endl;
    std::cout << "bin_count: " << holder->bin_count() << std::endl;
    std::cout << "first_beam_value: " << holder->first_beam_value() << std::endl;
    std::cout << "last_beam_value: " << holder->last_beam_value() << std::endl;

    SonarViewContext context(*holder);
    SonarView_initialize(context);
    SonarView_run(context);
}

int main(int argc, char **argv)
{

    sonarlog_target_tracking::ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv))
    {
        return -1;
    }

    DatasetInfo dataset_info = DatasetInfo(argument_parser.dataset_info_filename());
    sonarlog_target_tracking::DatasetInfoEntry entry = dataset_info.positive_entries()[0];

    if (entry.from_index == -1) entry.from_index = 0;
    entry.to_index = entry.from_index+1;

    SonarHolder sonar_holder;
    common::exec_samples_from_dataset_entry(entry, sample_receiver_callback, &sonar_holder);

    return 0;
}
