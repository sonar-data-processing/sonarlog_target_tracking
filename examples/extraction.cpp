#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <boost/filesystem.hpp>
#include <base/samples/Sonar.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include <sonar_processing/Preprocessing.hpp>
#include "ArgumentParser.hpp"
#include "Common.hpp"
#include "DatasetInfo.hpp"

using namespace sonarlog_target_tracking;
using namespace sonar_processing;

const static std::string FOLDERS[] =
{
    "01_source",
    "02_enhanced",
    "03_denoised",
    "04_preprocessed"
};

enum ExtractionFolders {
    kSource = 0,
    kEnhanced,
    kDenoised,
    kPreprocessed,
    kExtractionFoldersFinal
};

inline void write_image(
    const std::string& filepath,
    const cv::Mat& source)
{
    std::cout << "Saving: " << filepath << std::endl;
    cv::Mat out;
    if (source.type() == CV_32F)
        source.convertTo(out, CV_8U, 255);
    else
        source.copyTo(out);

    cv::cvtColor(out, out, CV_GRAY2BGR);

    cv::imwrite(filepath, out);
}

inline bool create_directory(const std::string& dirpath)
{
    boost::filesystem::path dir(dirpath);
    if (common::file_exists(dirpath) ||
        boost::filesystem::create_directory(dir)) {
        return true;
    }
    return false;
}

inline std::string file_join(const std::string& src0, const std::string& src1) {
    return (src0.at(src0.size()-1) == '/') ? src0 + src1 : src0 + "/" + src1;
}

inline std::string get_output_directory(const DatasetInfo& dataset_info, std::string folder_type = "train")
{
    create_directory(dataset_info.extraction_settings().extract_directory);

    std::string dirpath;
    if (dataset_info.extraction_settings().extract_rotation_norm) {
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "rotated");
        create_directory(dirpath);
    }
    else {
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "original");
        create_directory(dirpath);
    }
    return file_join(dirpath, folder_type);
}

inline std::string get_folder(
    const DatasetInfo& dataset_info,
    const ExtractionFolders k,
    std::string folder_type = "train",
    std::string suffix = "")
{
    return file_join(get_output_directory(dataset_info, folder_type), FOLDERS[k]+suffix);
}

inline void create_directories(const DatasetInfo& dataset_info, std::string folder_type = "train")
{
    std::string output_directory = get_output_directory(dataset_info, folder_type);
    create_directory(output_directory);

    if (dataset_info.extraction_settings().extract_annotation_orientation) {
        create_directory(file_join(output_directory, "annotation_orientation"));
    }

    if (dataset_info.extraction_settings().save_source_image) {
        create_directory(file_join(output_directory, FOLDERS[kSource]));
    }

    if (dataset_info.extraction_settings().save_enhanced_image) {
        create_directory(file_join(output_directory, FOLDERS[kEnhanced]));
    }

    if (dataset_info.extraction_settings().save_denoised_image) {
        create_directory(file_join(output_directory, FOLDERS[kDenoised]));
    }

    if (dataset_info.extraction_settings().save_preprocessed_image) {
        create_directory(file_join(output_directory, FOLDERS[kPreprocessed]));
    }
}

inline std::string get_filename(int index, std::string ext = ".png")
{
    std::stringstream ss;
    ss << "sample-";
    ss << index;
    ss << ext;
    return ss.str();
}

inline void save_file_list(const std::string& filepath, const std::vector<std::string>& files)
{
  std::ofstream f;
  f.open(filepath.c_str());
  for (size_t i = 0; i < files.size(); i++) {
      f << files[i]+"\n";
  }
  f.close();
}

inline void save_annotations(const std::string& filepath, const std::vector<cv::Rect>& bounding_boxs)
{
    std::ofstream f;
    f.open(filepath.c_str());
    for (size_t i = 0; i < bounding_boxs.size(); i++) {
      f << bounding_boxs[i].x;
      f << ",";
      f << bounding_boxs[i].y;
      f << ",";
      f << bounding_boxs[i].width;
      f << ",";
      f << bounding_boxs[i].height;
      f << "\n";
    }
    f.close();
}

inline void write_yolo_annotation(const std::string& filepath, const cv::Size& size, int id, cv::Rect bbox, double angle)
{


    double fx = 1.0/size.width;
    double fy = 1.0/size.height;
    double x = (bbox.x+bbox.width/2.0)*fx;
    double y = (bbox.y+bbox.height/2.0)*fy;
    double w = bbox.width*fx;
    double h = bbox.height*fy;

    char buffer[1024];
    if (angle != std::numeric_limits<double>::quiet_NaN())
        snprintf(buffer, 1024, "%d %f %f %f %f %f\n", id, x, y, w, h, angle);
    else
        snprintf(buffer, 1024, "%d %f %f %f %f\n", id, x, y, w, h);

    std::ofstream f;
    f.open(filepath.c_str());
    f << buffer;
    f.close();
}

inline void write_yolo_annotation(
    const std::string& filepath, const cv::Size& size, int id, cv::Rect bbox)
{
    write_yolo_annotation(filepath, size, id, bbox, std::numeric_limits<double>::quiet_NaN());
}

inline void write_training_files(const std::string& dirpath, std::vector<std::string> files)
{
    int test_count = 100.0 / 10.0;

    std::ofstream file_train;
    std::ofstream file_test;
    file_train.open(file_join(dirpath, "train.txt").c_str());
    file_test.open(file_join(dirpath, "test.txt").c_str());

    int cnt = 1;
    for (size_t i = 0; i < files.size(); i++) {
        if (cnt == test_count) {
            file_test << files[i]+"\n";
            cnt = 1;
        }
        else {
            file_train << files[i]+"\n";
            cnt++;
        }
    }
}

void rotate(const cv::Mat& src, cv::Mat& dst, double angle, const cv::Rect roi = cv::Rect(0, 0, -1, -1))
{
    cv::Mat rot;
    image_util::rotate(src, rot, angle, cv::Point2f(src.cols/2, src.rows/2));

    if (roi.width > 0 && roi.height > 0) {
        rot(roi).copyTo(dst);
        return;
    }

    rot.copyTo(dst);
}

cv::Rect find_mask_rotated_roi(const cv::Mat& src_mask, double angle)
{
    cv::Mat mask;
    rotate(src_mask, mask, angle);
    return image_util::bounding_rect(mask);
}

void extraction(
    const std::vector<base::samples::Sonar>& samples,
    const std::vector<std::vector<cv::Point> >& annotations,
    const DatasetInfo& dataset_info,
    std::string folder_type = "train")
{

    SonarHolder sonar_holder;
    SonarImagePreprocessing sonar_image_preprocessing;

    std::vector<std::string> files[kExtractionFoldersFinal];
    std::vector<cv::Rect> bounding_boxs;
    bool save_image[kExtractionFoldersFinal];

    save_image[kSource] = dataset_info.extraction_settings().save_source_image;
    save_image[kEnhanced] = dataset_info.extraction_settings().save_enhanced_image;
    save_image[kDenoised] = dataset_info.extraction_settings().save_denoised_image;
    save_image[kPreprocessed] = dataset_info.extraction_settings().save_preprocessed_image;

    cv::Size image_size = dataset_info.preprocessing_settings().image_max_size;

    common::load_preprocessing_settings(
        dataset_info.preprocessing_settings(),
        sonar_image_preprocessing);

    for (size_t i = 0; i < samples.size(); i++) {
        if (annotations[i].empty()) continue;

        std::vector<cv::Point> annotation = annotations[i];

        common::load_sonar_holder(samples[i], sonar_holder, image_size);

        if (image_size != cv::Size(-1, -1)) {
            image_util::resize_points(annotation, annotation, sonar_holder.cart_width_factor(), sonar_holder.cart_height_factor());
        }

        cv::Mat preprocessed_image;
        cv::Mat preprocessed_mask;
        sonar_image_preprocessing.Apply(
            sonar_holder.cart_image(),
            sonar_holder.cart_image_mask(),
            preprocessed_image,
            preprocessed_mask);

        std::string filename = get_filename(i);
        std::string source_filepath = file_join(get_folder(dataset_info, kSource, folder_type), filename);
        std::string enhanced_filepath = file_join(get_folder(dataset_info, kEnhanced, folder_type), filename);
        std::string denoised_filepath = file_join(get_folder(dataset_info, kDenoised,folder_type), filename);
        std::string preprocessed_filepath = file_join(get_folder(dataset_info, kPreprocessed, folder_type), filename);

        files[kSource].push_back(source_filepath);
        files[kEnhanced].push_back(enhanced_filepath);
        files[kDenoised].push_back(denoised_filepath);
        files[kPreprocessed].push_back(preprocessed_filepath);

        cv::Mat annotation_mask;
        image_util::draw_mask_from_points(image_size, annotation, annotation_mask);

        cv::RotatedRect rotated_rect = cv::minAreaRect(annotation);
        double annotation_angle = (rotated_rect.size.width>=rotated_rect.size.height) ? rotated_rect.angle : rotated_rect.angle+90;

        cv::Rect bbox;
        cv::Mat source_image;
        cv::Mat enhanced_image;
        cv::Mat denoised_image;

        if (dataset_info.extraction_settings().extract_rotation_norm) {
            cv::Rect roi = find_mask_rotated_roi(sonar_holder.cart_image_mask(), annotation_angle);
            rotate(annotation_mask, annotation_mask, annotation_angle, roi);
            rotate(sonar_holder.cart_image(), source_image, annotation_angle, roi);
            rotate(sonar_image_preprocessing.enhanced(), enhanced_image, annotation_angle, roi);
            rotate(sonar_image_preprocessing.denoised(), denoised_image, annotation_angle, roi);
            rotate(preprocessed_image, preprocessed_image, annotation_angle, roi);

            annotation = preprocessing::find_biggest_contour(annotation_mask);
            bbox = image_util::bounding_rect(annotation_mask);
            rotated_rect = cv::minAreaRect(annotation);
            annotation_angle = (rotated_rect.size.width>=rotated_rect.size.height) ? rotated_rect.angle : rotated_rect.angle+90;
        }
        else {
            source_image = sonar_holder.cart_image();
            enhanced_image = sonar_image_preprocessing.enhanced();
            denoised_image = sonar_image_preprocessing.denoised();
            bbox = image_util::bounding_rect(annotation_mask);
        }

        bounding_boxs.push_back(bbox);

        if (dataset_info.extraction_settings().save_source_image) write_image(source_filepath, source_image);
        if (dataset_info.extraction_settings().save_enhanced_image) write_image(enhanced_filepath, enhanced_image);
        if (dataset_info.extraction_settings().save_denoised_image) write_image(denoised_filepath, denoised_image);
        if (dataset_info.extraction_settings().save_preprocessed_image) write_image(preprocessed_filepath, preprocessed_image);

        if (dataset_info.extraction_settings().extract_yolo_inputs) {
             for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
                 if (!save_image[k]) continue;
                 std::string filepath = file_join(get_folder(dataset_info, (ExtractionFolders)k, folder_type), get_filename(i, ".txt"));
                 write_yolo_annotation(filepath, image_size, dataset_info.extraction_settings().class_id, bbox);
             }
        }

        if (dataset_info.extraction_settings().extract_annotation_orientation) {
            std::string dirpath = file_join(get_output_directory(dataset_info, folder_type), "annotation_orientation");
            std::string filepath = file_join(dirpath, get_filename(i, ".txt"));
            write_yolo_annotation(filepath, image_size, dataset_info.extraction_settings().class_id, bbox, annotation_angle);
        }

        if (dataset_info.extraction_settings().show_source_image &&
            dataset_info.extraction_settings().save_source_image) {
            cv::imshow("source", source_image);
            cv::waitKey(10);
        }

        if (dataset_info.extraction_settings().show_enhanced_image &&
            dataset_info.extraction_settings().save_enhanced_image) {
            cv::imshow("enhanced", enhanced_image);
            cv::waitKey(10);
        }

        if (dataset_info.extraction_settings().show_denoised_image &&
            dataset_info.extraction_settings().save_denoised_image) {
            cv::imshow("denoised", denoised_image);
            cv::waitKey(10);
        }

        if (dataset_info.extraction_settings().show_preprocessed_image &&
            dataset_info.extraction_settings().save_preprocessed_image) {
            cv::imshow("preprocessed_image", preprocessed_image);
            cv::waitKey(10);
        }

        if (dataset_info.extraction_settings().show_annotation_image) {
            cv::Mat annotation_image;
            source_image.copyTo(annotation_image);
            image_util::draw_contour(annotation_image, annotation_image, cv::Scalar(0, 0, 255), annotation);
            cv::rectangle(annotation_image, bbox, cv::Scalar(0, 255, 0), 4);
            cv::circle(annotation_image, cv::Point(bbox.x+bbox.width/2, bbox.y+bbox.height/2), 5, cv::Scalar(0, 255, 0), 5);
            cv::imshow("annotation", annotation_image);
            cv::waitKey(10);
        }

    }

    for (size_t i = 0; i < kExtractionFoldersFinal; i++) {
        if (!save_image[i]) continue;

        std::string outfolder = get_folder(dataset_info, (ExtractionFolders)i, folder_type);
        std::string filename = file_join(outfolder, "list.txt");
        save_file_list(filename, files[i]);
        write_training_files(outfolder, files[i]);
    }
    save_annotations(file_join(get_output_directory(dataset_info, folder_type), "annotations.csv"), bounding_boxs);
}

int main(int argc, char **argv)
{
    ArgumentParser argument_parser;

    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    DatasetInfo dataset_info(argument_parser.dataset_info_filename());

    std::cout << "Extraction Settings: " << std::endl;
    std::cout << dataset_info.extraction_settings().to_string() << std::endl;

    create_directories(dataset_info, "train");

    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = dataset_info.positive_entries();

    std::vector<std::vector<cv::Point> > train_annotations;
    std::vector<base::samples::Sonar> train_samples;

    for (size_t i=0; i<entries.size(); i++) {
        sonarlog_target_tracking::common::load_training_data_from_dataset_entry(entries[i], train_samples, train_annotations);
    }

    extraction(train_samples, train_annotations, dataset_info, "train");
    return 0;
}
