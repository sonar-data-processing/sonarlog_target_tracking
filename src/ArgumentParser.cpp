#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "ArgumentParser.hpp"

using namespace boost;

namespace sonarlog_target_tracking {

ArgumentParser::ArgumentParser()
    : dataset_info_filename_("") {
}

ArgumentParser::~ArgumentParser() {
}

bool ArgumentParser::run(int argc, char **argv) {
    std::string app_name = boost::filesystem::basename(argv[0]);

    program_options::options_description desc("create video from sonar scan log");

    desc.add_options()
        ("dataset-info-filename,i", program_options::value<std::string>()->required(), "The YML file with the dataset information.")
        ("help,h", "show the command line description");

    program_options::positional_options_description pd;

    pd.add("dataset-info-filename", 1);

    program_options::variables_map vm;

    try {
        program_options::store(program_options::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return false;
        }

        if (vm.count("dataset-info-filename")) {
            dataset_info_filename_ = vm["dataset-info-filename"].as<std::string>();

            if (!file_exists(dataset_info_filename_)) {
                std::cerr << "ERROR: The dataset information file not found" << std::endl;
                return false;
            }
        }

        program_options::notify(vm);
    } catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return false;
    }

    return true;
}

bool ArgumentParser::file_exists(std::string filename) {

    if (!filesystem::exists(filename) &&
        !filesystem::exists(filesystem::path(filesystem::current_path()).string() + "/" + filename)){
        return false;
    }

    return true;
}

std::string ArgumentParser::get_filename(std::string file_path) {
    filesystem::path p(file_path);
    return p.stem().string();
}


} /* namespace sonarlog_target_tracking */
