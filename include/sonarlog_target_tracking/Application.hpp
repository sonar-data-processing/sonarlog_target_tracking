#ifndef Application_hpp
#define Application_hpp

#include <iostream>
#include <string>

#include "sonar_target_tracking/TargetTrack.hpp"

namespace sonarlog_target_tracking {

class Application {
public:

    int process_logfile(const std::string& filename, const std::string& stream_name);

    static Application* instance();

private:
    Application() {}
    ~Application() {}
    
    static Application *instance_;
};

} /* namespace sonarlog_target_tracking */



#endif /* Application_hpp */
