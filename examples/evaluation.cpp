#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "Common.hpp"
#include "DetectionEval.hpp"
#include "ROC.hpp"

int main(int argc, char **argv) {
    std::string app_name = boost::filesystem::basename(argv[0]);

    boost::program_options::options_description desc("sonarlog target tracking evaluation");

    desc.add_options()
        ("evaluation-filename,i", boost::program_options::value<std::string>()->required(), "The CSV filename with evaluation data.")
        ("help,h", "show the command line description");

    boost::program_options::positional_options_description pd;

    pd.add("evaluation-filename", 1);

    boost::program_options::variables_map vm;

    std::string evaluation_filename;

    try {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        if (vm.count("evaluation-filename")) {
            evaluation_filename = vm["evaluation-filename"].as<std::string>();

            if (!sonarlog_target_tracking::common::file_exists(evaluation_filename)) {
                std::cerr << "ERROR: The dataset information file not found" << std::endl;
                return -1;
            }
        }

        boost::program_options::notify(vm);
    } catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
    }

    sonarlog_target_tracking::DetectionEvalList detection_eval_list;
    detection_eval_list.csv_read(evaluation_filename);
    sonarlog_target_tracking::ROC roc(detection_eval_list.true_positive_rate_list, detection_eval_list.false_positive_rate_list);

    return 0;
}
