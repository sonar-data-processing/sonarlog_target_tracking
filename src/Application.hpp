#ifndef Application_hpp
#define Application_hpp

#include <iostream>
#include <string>
#include "rock_util/LogReader.hpp"
#include "sonar_processing/TargetTrack.hpp"
#include "base/Plot.hpp"

namespace sonarlog_target_tracking {

class Application {
public:

    void init(const std::string& filename, const std::string& stream_name);

    void process_logfile();

    void process_next_sample();

    void process_background_features_from_logfile(const std::string& filename, const std::string& stream_name);

    void plot(cv::Mat mat);

    base::Plot& plot() {
        return *(plot_.get());
    }

    static Application* instance();

private:

    Application() {}

    ~Application() {}

    std::auto_ptr<rock_util::LogReader> reader_;
    rock_util::LogStream stream_;

    static Application *instance_;
    std::auto_ptr<base::Plot> plot_;

    std::vector<double> background_features_;
};

} /* namespace sonarlog_target_tracking */



#endif /* Application_hpp */
