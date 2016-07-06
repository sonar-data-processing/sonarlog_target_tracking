#ifndef ArgumentParser_hpp
#define ArgumentParser_hpp

#include <string>

namespace sonarlog_target_tracking {

class ArgumentParser {
public:

    ArgumentParser();
    virtual ~ArgumentParser();

    std::string input_file() const {
        return input_file_;
    }

    std::string app_name() const {
        return app_name_;
    }

    std::string stream_name() const {
        return stream_name_;
    }

    bool run(int argc, char const *argv[]);

private:

    bool file_exists(std::string filename);
    std::string get_filename(std::string file_path);

    std::string input_file_;
    std::string stream_name_;
    std::string app_name_;

};

} /* namespace sonarlog_target_tracking */

#endif /* ArgumentParser_hpp */
