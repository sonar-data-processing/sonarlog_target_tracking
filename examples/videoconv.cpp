#include <iostream>
#include <rock_util/LogReader.hpp>
#include "ArgumentParser.hpp"
#include "Common.hpp"
#include "VideoConverterSettings.hpp"

using namespace sonarlog_target_tracking;

struct Context
{
    std::vector<std::vector<rock_util::LogReader*> > log_reader_list;
    std::vector<sonar_processing::SonarHolder> sonar_holder_list;
    std::vector<std::vector<int> > index_list;
    VideoConverterSettings settings;

};

void init(Context& context)
{
    VideoMatrixSettings vms = context.settings.video_matrix_settings();
    context.log_reader_list.resize(vms.data.size());
    context.sonar_holder_list.resize(vms.data.size());
    context.index_list.resize(vms.data.size());


    for (size_t cell_index = 0; cell_index < vms.data.size(); cell_index++) {
        if (!vms.data[cell_index].empty()) {
            context.log_reader_list[cell_index].resize(vms.data[cell_index].size(), NULL);
            context.index_list[cell_index].resize(vms.data[cell_index].size(), 0);
            for (size_t i = 0; i < vms.data[cell_index].size(); i++) {
                assert(common::file_exists(vms.data[cell_index][i].filename));
                context.log_reader_list[cell_index][i] = new rock_util::LogReader(vms.data[cell_index][i].filename);;
            }
        }
    }
}

void release(Context& context)
{
    for (size_t cell_index = 0; cell_index < context.log_reader_list.size(); cell_index++) {
        if (!context.log_reader_list[cell_index].empty()) {
            for (size_t i = 0; i < context.log_reader_list[cell_index].size(); i++) {
                delete context.log_reader_list[cell_index][i];
            }
        }
    }
}

void process(Context& context)
{
    VideoMatrixSettings vms = context.settings.video_matrix_settings();

    cv::Size isz = cv::Size(vms.image_width, vms.image_height);
    cv::Mat out = cv::Mat::zeros(cv::Size(vms.cols * isz.width, vms.rows * isz.height), CV_32FC1);

    cv::VideoWriter output_video;
    output_video.open(vms.output_file, CV_FOURCC('P','I','M','1'), 25, out.size(), true);

    bool done;
    do {
        done = true;
        for (size_t cell_index = 0; cell_index < context.log_reader_list.size(); cell_index++) {
            if (!context.log_reader_list[cell_index].empty()) {
                for (size_t i = 0; i < context.log_reader_list[cell_index].size(); i++) {
                    rock_util::LogStream stream = context.log_reader_list[cell_index][i]->stream(vms.data[cell_index][i].stream_name);
                    if (context.index_list[cell_index][i] < stream.total_samples()) {
                        base::samples::Sonar sample;
                        if (stream.read_sample<base::samples::Sonar>(sample, context.index_list[cell_index][i])) {

                            context.sonar_holder_list[cell_index].Reset(
                                sample.bins,
                                rock_util::Utilities::get_radians(sample.bearings),
                                sample.beam_width.getRad(),
                                sample.bin_count,
                                sample.beam_count,
                                isz
                            );

                            int row = cell_index / vms.cols;
                            int col = cell_index % vms.cols;
                            cv::Rect roi = cv::Rect(col * isz.width, row * isz.height, isz.width, isz.height);
                            context.sonar_holder_list[cell_index].cart_image().copyTo(out(roi));

                        }
                        context.index_list[cell_index][i]++;
                        done = false;
                        break;
                    }
                }
            }
        }
        cv::Mat out_8u;
        out.convertTo(out_8u, CV_8U, 255);

        cv::Mat frame;
        cv::cvtColor(out_8u, frame, CV_GRAY2BGR);
        output_video << frame;

        cv::imshow("result", frame);
        cv::waitKey(15);

    } while (!done);
}

int main(int argc, char **argv)
{

    ArgumentParser argument_parser;
    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    Context context;
    context.settings = VideoConverterSettings(argument_parser.videoconv_conf_filename());

    init(context);
    process(context);
    release(context);
    return 0;
}
