#include <iostream>
#include "sonarlog_target_tracking/ArgumentParser.hpp"
#include "sonarlog_target_tracking/Application.hpp"

using namespace sonarlog_target_tracking;

int main(int argc, char const *argv[]) {
    
    ArgumentParser argument_parser;
    if (argument_parser.run(argc, argv)) {
        std::cout << "Sonar's log processing" << std::endl;
        std::cout << "intput-file: " << argument_parser.input_file()  << std::endl;
        std::cout << "stream-name: " << argument_parser.stream_name() << std::endl;
        return Application::instance()->process_logfile(argument_parser.input_file(), 
                                                        argument_parser.stream_name());
    }
    
    return 0;
}
