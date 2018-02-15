#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "ArgumentParser.hpp"

using namespace boost;

namespace sonarlog_target_tracking {

ArgumentParser::ArgumentParser()
    : dataset_info_filename_("")
    , evaluation_settings_filename_("")
    , videoconv_conf_filename_("")
    , show_detection_result_(false)
{
}

ArgumentParser::~ArgumentParser() {
}

bool ArgumentParser::run(int argc, char **argv) {
    std::string app_name = boost::filesystem::basename(argv[0]);

    program_options::options_description desc(app_name);


    if (app_name == "sonarlog-evaluation") {
        desc.add_options()
            ("dataset-info-filename,i", program_options::value<std::string>()->required(), "The YML file with the dataset information.")
            ("evaluation-settings-filename,e", program_options::value<std::string>(), "The YML with evaluation filename.")
            ("show-detection-result,s", program_options::value<bool>(), "Show detection result.")
            ("help,h", "show the command line description");
    }
    else if (app_name == "sonarlog-videoconv") {
        desc.add_options()
            ("conf,c", program_options::value<std::string>()->required(), "The YML configuration file.")
            ("help,h", "show the command line description");
    }
    else {
        desc.add_options()
            ("dataset-info-filename,i", program_options::value<std::string>()->required(), "The YML file with the dataset information.")
            ("help,h", "show the command line description");
    }


    program_options::positional_options_description pd;

    pd.add("dataset-info-filename", 1);

    program_options::variables_map vm;

    try {
        program_options::store(program_options::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return false;
        }

        if (app_name == "sonarlog-videoconv") {
            if (vm.count("conf")) {
                videoconv_conf_filename_ = vm["conf"].as<std::string>();

                if (!file_exists(videoconv_conf_filename_)) {
                    std::cerr << "ERROR: The video configuration file does not exist" << std::endl;
                    return false;
                }
            }
        }
        else {

            if (vm.count("dataset-info-filename")) {
                dataset_info_filename_ = vm["dataset-info-filename"].as<std::string>();

                if (!file_exists(dataset_info_filename_)) {
                    std::cerr << "ERROR: The dataset information file not found" << std::endl;
                    return false;
                }
            }

            if (app_name == "sonarlog-evaluation") {
                if (vm.count("evaluation-settings-filename")) {
                    evaluation_settings_filename_ = vm["evaluation-settings-filename"].as<std::string>();

                    if (!file_exists(evaluation_settings_filename_)) {
                        std::cerr << "ERROR: The evaluation settings file not found" << std::endl;
                        return false;
                    }
                }

                if (vm.count("show-detection-result")) {
                    show_detection_result_ = vm["show-detection-result"].as<bool>();
                    std::cout << "show_detection_result: " << show_detection_result_ << std::endl;
                }
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
