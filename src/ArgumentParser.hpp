#ifndef ArgumentParser_hpp
#define ArgumentParser_hpp

#include <string>
#include <vector>

namespace sonarlog_target_tracking {

class ArgumentParser {
public:

    ArgumentParser();
    virtual ~ArgumentParser();

    std::string app_name() const {
        return app_name_;
    }

    std::string dataset_info_filename() const {
        return dataset_info_filename_;
    }

    std::string evaluation_settings_filename() const {
        return evaluation_settings_filename_;
    }

    bool show_detection_result() const {
        return show_detection_result_;
    }

    bool run(int argc, char **argv);


private:

    bool file_exists(std::string filename);
    std::string get_filename(std::string file_path);

    std::string app_name_;
    std::string dataset_info_filename_;
    std::string evaluation_settings_filename_;
    bool show_detection_result_;

};

} /* namespace sonarlog_target_tracking */

#endif /* ArgumentParser_hpp */
