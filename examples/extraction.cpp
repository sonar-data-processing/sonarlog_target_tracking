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

typedef std::vector<sonarlog_target_tracking::DatasetInfoEntry> DatasetInfoEntryList;
typedef std::vector<std::vector<cv::Point> > AnnotationList;

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

struct Context {
    size_t sample_count;
    DatasetInfo dataset_info;
    AnnotationList annotations;
    SonarHolder sonar_holder;
    SonarImagePreprocessing preprocessing;
    std::vector<std::string> files[kExtractionFoldersFinal];
    std::vector<cv::Rect> boxes;
    std::vector<cv::RotatedRect> rboxes;
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

inline std::string get_output_directory(const DatasetInfo& dataset_info, std::string folder_type = std::string())
{
    create_directory(dataset_info.extraction_settings().extract_directory);

    std::string dirpath;
    if (dataset_info.extraction_settings().extract_target_bbox)
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "bbox");
    else if (dataset_info.extraction_settings().extract_target_orientations)
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "orientations");
    else if (dataset_info.extraction_settings().extract_rotation_norm)
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "rotated");
    else
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "original");

    create_directory(dirpath);
    return file_join(dirpath, folder_type);
}

inline std::string get_folder(
    const DatasetInfo& dataset_info,
    const ExtractionFolders k,
    std::string folder_type = std::string(),
    std::string suffix = "")
{
    return file_join(get_output_directory(dataset_info, folder_type), FOLDERS[k]+suffix);
}

inline void create_directories(const DatasetInfo& dataset_info, std::string folder_type = std::string())
{
    std::string output_directory = get_output_directory(dataset_info, folder_type);
    create_directory(output_directory);

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

inline std::string get_filename(int index, std::string ext = ".png", std::string suffix = std::string())
{
    char buffer[256];
    snprintf(buffer, 256, "sample-%08d", index);

    std::stringstream ss;
    ss << buffer;
    if (!suffix.empty()) ss << suffix;
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

inline void save_bbox_annotations(
    const std::string& filepath,
    const std::vector<std::string>& files,
    const std::vector<cv::Rect>& bbox,
    const std::string& class_name)
{
    std::ofstream f;
    f.open(filepath.c_str());
    for (size_t i = 0; i < files.size(); i++) {
        char buffer[4096];
        float x1 = bbox[i].x;
        float y1 = bbox[i].y;
        float x2 = x1+bbox[i].width;
        float y2 = y1+bbox[i].height;
        snprintf(buffer, 4096, "%s,%0.2f,%0.2f,%0.2f,%0.2f,%s\n", files[i].c_str(), x1, y1, x2, y2, class_name.c_str());
        f << buffer;
    }
    f.close();
}

inline void save_rotated_bbox_annotations(
    const std::string& filepath,
    const std::vector<std::string>& files,
    const std::vector<cv::Rect>& bbox,
    const std::vector<cv::RotatedRect>& rbbox,
    const std::string& class_name)
{
    std::ofstream f;
    f.open(filepath.c_str());
    for (size_t i = 0; i < files.size(); i++) {
        char buffer[4096];
        float x1 = bbox[i].x;
        float y1 = bbox[i].y;
        float x2 = x1+bbox[i].width;
        float y2 = y1+bbox[i].height;
        float cx = rbbox[i].center.x;
        float cy = rbbox[i].center.y;
        float w = rbbox[i].size.width;
        float h = rbbox[i].size.height;
        float t = rbbox[i].angle;

        snprintf(buffer, 4096, "%s,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%s\n",
            files[i].c_str(),
            x1, y1, x2, y2,
            cx, cy, w, h, t,
            class_name.c_str());

        f << buffer;
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
    if (angle != angle)
        snprintf(buffer, 1024, "%d %f %f %f %f\n", id, x, y, w, h);
    else
        snprintf(buffer, 1024, "%d %f %f %f %f %f\n", id, x, y, w, h, angle);

    std::ofstream f;
    f.open(filepath.c_str());
    f << buffer;
    f.close();
}

inline void write_yolo_annotation(const std::string& filepath, const cv::Size& size, int id, cv::Rect bbox)
{
    write_yolo_annotation(filepath, size, id, bbox, std::numeric_limits<double>::quiet_NaN());
}

inline void write_bbox_annotation(const std::string& filepath, int id, const cv::Point2f center, const cv::Size2f size, float angle)
{
    char buffer[1024];
    snprintf(buffer, 1024, "%d %f %f %f %f %f\n", id, center.x, center.y, size.width, size.height, angle);
    std::ofstream f;
    f.open(filepath.c_str());
    f << buffer;
    f.close();
}

inline float distance(cv::Point2f pt0, cv::Point2f pt1)
{
    float dx = pt1.x-pt0.x;
    float dy = pt1.y-pt0.y;
    return sqrt(dx * dx + dy * dy);
}

inline void write_rbbox_annotation(const std::string& filepath, const cv::Size& image_size, int id, const cv::RotatedRect& rbbox)
{
    double fx = 1.0/image_size.width;
    double fy = 1.0/image_size.height;

    double scale_factor = std::min(fx, fy);

    cv::Point2f v[4];
    rbbox.points(v);

    for (int i = 0; i < 4; i++) {
        v[i].x *= scale_factor;
        v[i].y *= scale_factor;
    }

    float w = distance(v[0], v[3]);
    float h = distance(v[0], v[1]);

    cv::Point2f center = cv::Point2f(rbbox.center.x*scale_factor, rbbox.center.y*scale_factor);
    cv::Size2f size = cv::Size2f(w, h);
    write_bbox_annotation(filepath, id, center, size, rbbox.angle);
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

inline cv::RotatedRect load_rotated_rect(cv::InputArray src) {

    cv::RotatedRect bbox = cv::minAreaRect(src);

    if (bbox.size.width<bbox.size.height) {
        float w = bbox.size.width;
        bbox.size.width = bbox.size.height;
        bbox.size.height = w;
        bbox.angle += 90;
    }

    if (bbox.angle < 0) {
        bbox.angle = bbox.angle+180;
    }

    return bbox;

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
    std::string folder_type = std::string())
{

    SonarHolder sonar_holder;
    SonarImagePreprocessing sonar_image_preprocessing;

    if (folder_type.empty()) folder_type = dataset_info.extraction_settings().class_name;

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

                 std::string dirpath = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
                 std::string filepath = file_join(dirpath, get_filename(i, ".txt"));

                 write_yolo_annotation(filepath, image_size, dataset_info.extraction_settings().class_id, bbox);

                 if (dataset_info.extraction_settings().extract_annotation_mask) {
                     filepath = file_join(dirpath, get_filename(i, ".png", "-mask"));
                     cv::imwrite(filepath, annotation_mask);
                 }

                 if (dataset_info.extraction_settings().extract_annotation_orientation) {
                     filepath = file_join(dirpath, get_filename(i, ".txt", "-rot"));
                     write_yolo_annotation(filepath, image_size, dataset_info.extraction_settings().class_id, bbox, annotation_angle);
                 }
             }
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

void rotate_all(
    const SonarHolder& holder,
    const SonarImagePreprocessing& preprocessing,
    const cv::Mat& preprocessed_image,
    const cv::Rect& roi,
    double t,
    cv::Mat res[kExtractionFoldersFinal])
{
    rotate(holder.cart_image(), res[kSource], t, roi);
    rotate(preprocessing.enhanced(), res[kEnhanced], t, roi);
    rotate(preprocessing.denoised(), res[kDenoised], t, roi);
    rotate(preprocessed_image, res[kPreprocessed], t, roi);
}



void extraction_orientations(
    const std::vector<base::samples::Sonar>& samples,
    const std::vector<std::vector<cv::Point> >& annotations,
    const DatasetInfo& dataset_info,
    std::string folder_type = std::string())
{
    SonarHolder sonar_holder;
    SonarImagePreprocessing sonar_image_preprocessing;

    if (folder_type.empty()) folder_type = dataset_info.extraction_settings().class_name;

    common::load_preprocessing_settings(
        dataset_info.preprocessing_settings(),
        sonar_image_preprocessing);

    cv::Size image_size = dataset_info.preprocessing_settings().image_max_size;

    double theta_step = dataset_info.extraction_settings().extract_target_orientations_step;

    bool save_image[kExtractionFoldersFinal];

    save_image[kSource] = dataset_info.extraction_settings().save_source_image;
    save_image[kEnhanced] = dataset_info.extraction_settings().save_enhanced_image;
    save_image[kDenoised] = dataset_info.extraction_settings().save_denoised_image;
    save_image[kPreprocessed] = dataset_info.extraction_settings().save_preprocessed_image;

    std::vector<std::vector<std::string> > out_directories;
    out_directories.resize(kExtractionFoldersFinal);

    std::vector<std::vector<std::string> > files;
    files.resize(kExtractionFoldersFinal);

    for (double theta = 0; theta <= 180; theta+=theta_step) {
        char buffer[256];
        snprintf(buffer, 256, "t%08d", (int)theta);
        std::string dst_dir = buffer;
        for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
            if (save_image[k]) {
                std::string root_dir = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
                std::string out_dir = file_join(root_dir, dst_dir);
                create_directory(out_dir);
                out_directories[k].push_back(out_dir);
            }
        }
    }

    for (size_t i = 0; i < samples.size(); i++) {
        if (annotations[i].empty()) continue;
        printf("%d of %d samples...\n", i+1, samples.size());
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

        cv::Mat annotation_mask;
        image_util::draw_mask_from_points(image_size, annotation, annotation_mask);

        cv::RotatedRect rotated_rect = cv::minAreaRect(annotation);
        double annotation_angle = (rotated_rect.size.width>=rotated_rect.size.height) ? rotated_rect.angle : rotated_rect.angle+90;

        cv::Mat results[kExtractionFoldersFinal];

        std::string filename = get_filename(i);

        if (!dataset_info.extraction_settings().extract_target_orientations_keep) {
            cv::Rect roi = find_mask_rotated_roi(sonar_holder.cart_image_mask(), annotation_angle);
            rotate(annotation_mask, annotation_mask, annotation_angle, roi);
            rotate_all(sonar_holder, sonar_image_preprocessing, preprocessed_image, roi, annotation_angle, results);

            int idx = 0;
            for (double theta = 0; theta <= 180; theta+=theta_step) {
                std::cout << "Idx: " << idx << " Theta: " << theta << std::endl;

                cv::Mat rot_annotation_mask;
                rotate(annotation_mask, rot_annotation_mask, theta);
                cv::Rect rc =  image_util::bounding_rect(rot_annotation_mask);

                for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
                    if (save_image[k]) {
                        cv::Mat rotated;
                        rotate(results[k], rotated, theta);
                        std::string filepath = file_join(out_directories[k][idx], filename);
                        files[k].push_back(filepath);

                        cv::Mat out;
                        rotated(rc).copyTo(out);
                        write_image(filepath, out);
                    }
                }
                idx++;
            }
        }
        else {
            cv::Mat canvas;
            cv::cvtColor(sonar_holder.cart_image(), canvas, CV_GRAY2BGR);

            cv::Rect orig_rc = image_util::bounding_rect(annotation_mask);
            cv::Rect orig_roi = image_util::bounding_rect(sonar_holder.cart_image_mask());

            cv::Mat orig_results[kExtractionFoldersFinal];
            rotate_all(sonar_holder, sonar_image_preprocessing, preprocessed_image, orig_roi, 360.0, orig_results);

            image_util::draw_rotated_rect(canvas, cv::Scalar(0, 0, 255), rotated_rect);
            cv::rectangle(canvas, orig_rc, cv::Scalar(0, 0, 255), 3);

            cv::imshow("canvas", canvas);
            cv::waitKey(15);

            cv::Rect roi = find_mask_rotated_roi(sonar_holder.cart_image_mask(), annotation_angle);
            rotate(annotation_mask, annotation_mask, annotation_angle, roi);
            rotate_all(sonar_holder, sonar_image_preprocessing, preprocessed_image, roi, annotation_angle, results);

            double theta_step2 = theta_step / 2;
            double angle = 180-annotation_angle;
            double last_theta = 180+theta_step2;

            if (angle >= last_theta) angle = angle-last_theta;

            int idx = 0;
            for (double t = 0; t <= 180; t+=theta_step) {
                double theta = 180-t;
                double theta_begin = theta-theta_step2;
                double theta_end = theta+theta_step2;

                std::cout << "theta: " << theta << std::endl;

                if (angle >= theta_begin && angle < theta_end) {
                    std::cout << "angle: " << angle << std::endl;
                    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
                        if (save_image[k]) {
                            std::string filepath = file_join(out_directories[k][out_directories[k].size()-idx-1], filename);
                            files[k].push_back(filepath);
                            cv::Mat out;
                            orig_results[k](orig_rc).copyTo(out);
                            write_image(filepath, out);
                        }
                    }

                }
                else {
                    cv::Mat rot_mask;
                    rotate(annotation_mask, rot_mask, -t);
                    cv::Rect rot_rc =  image_util::bounding_rect(rot_mask);

                    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
                        if (save_image[k]) {
                            cv::Mat rot;
                            rotate(results[k], rot, -t);

                            std::string filepath = file_join(out_directories[k][out_directories[k].size()-idx-1], filename);
                            files[k].push_back(filepath);

                            cv::Mat out;
                            rot(rot_rc).copyTo(out);
                            write_image(filepath, out);
                        }

                    }
                }
                idx++;
            }
        }
    }

    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (save_image[k]) {
            std::string root_dir = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
            std::string filename = file_join(root_dir, "list.txt");
            save_file_list(filename, files[k]);
            write_training_files(root_dir,files[k]);
        }
    }


}

void save_bbox_data(
    const cv::Mat image,
    const cv::Rect& rect,
    const cv::RotatedRect& rbox,
    const size_t index,
    const DatasetInfo& dataset_info,
    const std::string& root_dir,
    std::vector<std::string>& files,
    std::string suffix = std::string())
{
    std::string image_filepath = file_join(root_dir, get_filename(index, ".png", suffix));
    files.push_back(image_filepath);
    write_image(image_filepath, image(rect));
    write_bbox_annotation(
        file_join(root_dir, get_filename(index, ".txt", suffix)),
        dataset_info.extraction_settings().class_id,
        cv::Point2f(rbox.center.x-rect.x, rbox.center.y-rect.y),
        rbox.size,
        rbox.angle);
}

void save_images_bbox_data(
    const cv::Mat images[kExtractionFoldersFinal],
    const cv::Rect& rect,
    const cv::RotatedRect& rbox,
    const bool save_image[kExtractionFoldersFinal],
    const size_t index,
    const DatasetInfo& dataset_info,
    std::vector<std::string> (&files)[kExtractionFoldersFinal],
    std::string folder_type = std::string(),
    std::string suffix = std::string())
{
    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (save_image[k]) {
            std::string root_dir = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
            save_bbox_data(images[k], rect, rbox, index, dataset_info, root_dir, files[k], suffix);
        }
    }
}

void rotate_images(
    const cv::Mat images[kExtractionFoldersFinal],
    cv::Mat rot_images[kExtractionFoldersFinal],
    const bool save_image[kExtractionFoldersFinal],
    const float theta,
    cv::Rect roi = cv::Rect(0, 0, -1, -1))
{
    for (size_t k = 0; k < kExtractionFoldersFinal; k++)
        if (save_image[k]) rotate(images[k], rot_images[k], theta, roi);
}

void save_bbox_data_multiple_angles(
    const cv::Mat images[kExtractionFoldersFinal],
    const cv::Mat gt_mask,
    const cv::Rect& rect,
    const cv::RotatedRect& rbox,
    const bool save_image[kExtractionFoldersFinal],
    const size_t index,
    const DatasetInfo& dataset_info,
    std::vector<std::string> (&files)[kExtractionFoldersFinal],
    std::string folder_type = std::string())
{
    const float theta_step = 10;
    const float gt_angle = rbox.angle;

    cv::Mat rot_images[kExtractionFoldersFinal];

    for (float theta = 0; theta <= 180; theta +=theta_step) {
        float theta_rot = gt_angle-theta;
        cv::Mat rot_gt_mask;
        rotate(gt_mask, rot_gt_mask, theta_rot);
        rotate_images(images, rot_images, save_image, theta_rot);


        // find contour
        std::vector<cv::Point> contour = preprocessing::find_biggest_contour(rot_gt_mask);

        // load ground truth rotated rect
        cv::RotatedRect rot_rbox = load_rotated_rect(contour);

        // load bounding box
        cv::Rect rot_rect =  image_util::bounding_rect(rot_gt_mask);

        char buffer[256];
        snprintf(buffer, 256, "-%04d", (int)theta);

        save_images_bbox_data(
            rot_images, rot_rect, rot_rbox, save_image,
            index, dataset_info, files, folder_type, buffer);
    }

}

void extraction_bbox(
    const std::vector<base::samples::Sonar>& samples,
    const std::vector<std::vector<cv::Point> >& annotations,
    const DatasetInfo& dataset_info,
    std::string folder_type = std::string())
{
    SonarHolder sonar_holder;
    SonarImagePreprocessing sonar_image_preprocessing;

    if (folder_type.empty()) folder_type = dataset_info.extraction_settings().class_name;


    cv::Mat images[kExtractionFoldersFinal];
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
        printf("%d of %d samples...\n", i+1, samples.size());
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


        images[kSource] = sonar_holder.cart_image();
        images[kEnhanced] = sonar_image_preprocessing.enhanced();
        images[kDenoised] = sonar_image_preprocessing.denoised();
        images[kPreprocessed] = preprocessed_image;

        // ground truth mask
        cv::Mat gt_mask;
        image_util::draw_mask_from_points(image_size, annotation, gt_mask);

        // load ground truth rotated rect
        cv::RotatedRect rotated_rect = load_rotated_rect(annotation);

        // returns ground truth bounding rect
        cv::Rect rect = image_util::bounding_rect(gt_mask);

        cv::imshow("bbox", images[kSource](rect));
        cv::waitKey(15);

        // save bounding box data
        save_images_bbox_data(images, rect, rotated_rect, save_image, i, dataset_info, files, folder_type);

        // save bounding box multiple angles
        save_bbox_data_multiple_angles(images, gt_mask, rect, rotated_rect, save_image, i, dataset_info, files, folder_type);
    }

    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (save_image[k]) {
            std::string root_dir = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
            std::string filename = file_join(root_dir, "list.txt");
            save_file_list(filename, files[k]);
            write_training_files(root_dir,files[k]);
        }
    }


}

void load_samples(
    const DatasetInfo& dataset_info,
    std::vector<base::samples::Sonar>& samples,
    std::vector<std::vector<cv::Point> >& annotations)
{
    std::vector<sonarlog_target_tracking::DatasetInfoEntry> entries = dataset_info.positive_entries();

    if (dataset_info.extraction_settings().training_samples) {
        for (size_t i=0; i<entries.size(); i++) {
            sonarlog_target_tracking::common::load_training_data_from_dataset_entry(entries[i], samples, annotations);
        }
    }
    else {
        for (size_t i=0; i<entries.size(); i++) {
            sonarlog_target_tracking::common::load_samples_from_dataset_entry(entries[i], samples, annotations);
        }
    }
}

void load_save_options(const ExtractionSettings& extract_settings, bool (&save_options)[kExtractionFoldersFinal])
{
    save_options[kSource] = extract_settings.save_source_image;
    save_options[kEnhanced] = extract_settings.save_enhanced_image;
    save_options[kDenoised] = extract_settings.save_denoised_image;
    save_options[kPreprocessed] = extract_settings.save_preprocessed_image;
}

void apply_preprocessing(
    const cv::Mat& source_image,
    const cv::Mat& source_mask,
    SonarImagePreprocessing& preprocessing,
    cv::Mat (&images)[kExtractionFoldersFinal])
{
    cv::Mat preprocessed_image;
    cv::Mat preprocessed_mask;
    preprocessing.Apply(source_image, source_mask, preprocessed_image, preprocessed_mask);

    images[kSource] = source_image;
    images[kEnhanced] = preprocessing.enhanced();
    images[kDenoised] = preprocessing.denoised();
    images[kPreprocessed] = preprocessed_image;
}

void load_destination_filepath(
    const DatasetInfo& dataset_info,
    const std::string& filename,
    std::string (&filepaths)[kExtractionFoldersFinal])
{
    std::string class_name = dataset_info.extraction_settings().class_name;
    filepaths[kSource] = file_join(get_folder(dataset_info, kSource, class_name), filename);
    filepaths[kEnhanced] = file_join(get_folder(dataset_info, kEnhanced, class_name), filename);
    filepaths[kDenoised] = file_join(get_folder(dataset_info, kDenoised,class_name), filename);
    filepaths[kPreprocessed] = file_join(get_folder(dataset_info, kPreprocessed, class_name), filename);
}

void save_results(
    const cv::Mat (&images)[kExtractionFoldersFinal],
    const std::string (&filepaths)[kExtractionFoldersFinal],
    const bool (&save_options)[kExtractionFoldersFinal],
    const Context& context,
    const cv::Rect& bbox,
    const cv::RotatedRect& rbbox,
    const std::string suffix = std::string())
{
    std::string class_name = context.dataset_info.extraction_settings().class_name;
    int class_id = context.dataset_info.extraction_settings().class_id;

    size_t index = context.sample_count;

    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (!save_options[k]) continue;

        std::string dirpath = get_folder(context.dataset_info, (ExtractionFolders)k, class_name);
        std::string yolo_annotation_filepath = file_join(dirpath, get_filename(index, ".txt", suffix));

        std::string rbbox_suffix = suffix + "-rot";
        std::string rbbox_annotation_filepath = file_join(dirpath, get_filename(index, ".txt", rbbox_suffix));

        write_image(filepaths[k], images[k]);
        write_yolo_annotation(yolo_annotation_filepath, images[k].size(), class_id, bbox);
        write_rbbox_annotation(rbbox_annotation_filepath, images[k].size(), class_id, rbbox);
    }
}

void save_images_and_annotations(
    Context& context,
    const cv::Mat (&images)[kExtractionFoldersFinal],
    const cv::Mat& annotation_mask,
    const std::vector<cv::Point>& annotation,
    const bool (&save_options)[kExtractionFoldersFinal],
    std::string suffix = std::string())
{
    std::string filename = get_filename(context.sample_count, ".png", suffix);
    std::string filepaths[kExtractionFoldersFinal];
    load_destination_filepath(context.dataset_info, filename, filepaths);

    cv::Rect bbox = image_util::bounding_rect(annotation_mask);
    cv::RotatedRect rbbox = load_rotated_rect(annotation);

    save_results(images, filepaths, save_options, context, bbox, rbbox, suffix);

    context.boxes.push_back(bbox);
    context.rboxes.push_back(rbbox);
    for (size_t k = 0; k < kExtractionFoldersFinal; k++)
        context.files[k].push_back(filepaths[k]);


}

void extract_sample(
    Context& context,
    const cv::Mat& source_image,
    const cv::Mat& source_mask,
    const std::vector<cv::Point>& annotation,
    std::string suffix = std::string())
{
    cv::Mat annotation_mask;
    image_util::draw_mask_from_points(source_image.size(), annotation, annotation_mask);

    cv::Mat images[kExtractionFoldersFinal];
    apply_preprocessing(source_image, source_mask, context.preprocessing, images);

    bool save_options[kExtractionFoldersFinal];
    load_save_options(context.dataset_info.extraction_settings(), save_options);

    save_images_and_annotations(context, images, annotation_mask, annotation, save_options, suffix);
}

void extract_sample_multiple_angles(
    Context& context,
    const cv::Mat& source_image,
    const cv::Mat& source_mask,
    const std::vector<cv::Point>& annotation)
{
    const float theta_step = context.dataset_info.extraction_settings().extract_target_orientations_step;

    cv::Mat gt_mask;
    image_util::draw_mask_from_points(source_image.size(), annotation, gt_mask);

    cv::Mat images[kExtractionFoldersFinal];
    apply_preprocessing(source_image, source_mask, context.preprocessing, images);

    bool save_options[kExtractionFoldersFinal];
    load_save_options(context.dataset_info.extraction_settings(), save_options);

    size_t cnt = 0;
    for (float theta = -90; theta <= 90; theta +=theta_step) {
        cv::Rect roi = find_mask_rotated_roi(source_mask, theta);

        cv::Mat rot_mask;
        cv::Mat rot_gt_mask;
        cv::Mat rot_images[kExtractionFoldersFinal];

        rotate(source_mask, rot_mask, theta, roi);
        rotate(gt_mask, rot_gt_mask, theta, roi);

        rotate_images(images, rot_images, save_options, theta, roi);

        // find contour
        std::vector<cv::Point> rot_annotation = preprocessing::find_biggest_contour(rot_gt_mask);
        char buffer[32];
        snprintf(buffer, 32, "-%04d", (int)cnt);

        save_images_and_annotations(context, rot_images, rot_gt_mask, rot_annotation, save_options, buffer);

        cnt++;
    }
}

void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data)
{
    printf("Current sample: %d\n", sample_index);
    Context *pContext = reinterpret_cast<Context*>(user_data);

    if (pContext->annotations.empty() || pContext->annotations[sample_index].empty()) {
        printf("There is no annotation for sample: %d\n", sample_index);
        return;
    }

    cv::Size image_size = pContext->dataset_info.preprocessing_settings().image_max_size;
    common::load_sonar_holder(sample, pContext->sonar_holder, image_size);

    std::vector<cv::Point> annotation = pContext->annotations[sample_index];

    if (image_size != cv::Size(-1, -1)) {
        image_util::resize_points(
            annotation,
            annotation,
            pContext->sonar_holder.cart_width_factor(),
            pContext->sonar_holder.cart_height_factor());
    }

    const cv::Mat& source_image = pContext->sonar_holder.cart_image();
    const cv::Mat& source_mask = pContext->sonar_holder.cart_image_mask();

    extract_sample(*pContext, source_image, source_mask, annotation);

    if (pContext->dataset_info.extraction_settings().extract_target_orientations)
        extract_sample_multiple_angles(*pContext, source_image, source_mask, annotation);

    pContext->sample_count++;
}

void load_log_annotation(Context& context, sonarlog_target_tracking::DatasetInfoEntry& entry)
{
    context.annotations.clear();
    if (!entry.annotation_filename.empty() && common::file_exists(entry.annotation_filename)) {
        common::load_log_annotation(entry.annotation_filename, entry.annotation_name, context.annotations);
    }
}

void exec_samples(Context& context)
{
    DatasetInfoEntryList entries = context.dataset_info.positive_entries();

    common::load_preprocessing_settings(
        context.dataset_info.preprocessing_settings(),
        context.preprocessing);

    context.boxes.clear();
    context.rboxes.clear();

    for (size_t k = 0; k < kExtractionFoldersFinal; k++)
        context.files[k].clear();

    context.sample_count = 0;

    if (context.dataset_info.extraction_settings().training_samples) {
        for (size_t i=0; i<entries.size(); i++) {
            load_log_annotation(context, entries[i]);
            common::exec_training_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
        }
    }
    else {
        for (size_t i=0; i<entries.size(); i++) {
            load_log_annotation(context, entries[i]);
            common::exec_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
        }
    }

    bool save_options[kExtractionFoldersFinal];
    load_save_options(context.dataset_info.extraction_settings(), save_options);

    std::string class_name = context.dataset_info.extraction_settings().class_name;
    std::string outfolder = get_output_directory(context.dataset_info, class_name);

    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (!save_options[k]) continue;
        std::string bbox_annotation_filepath = file_join(outfolder, FOLDERS[k]+"_bbox_annotations.txt");
        std::string rotated_bbox_annotation_filepath = file_join(outfolder, FOLDERS[k]+"_rotated_bbox_annotations.txt");

        save_bbox_annotations(
            bbox_annotation_filepath,
            context.files[k],
            context.boxes,
            class_name);

        save_rotated_bbox_annotations(
            rotated_bbox_annotation_filepath,
            context.files[k],
            context.boxes,
            context.rboxes,
            class_name);
    }

}


int main(int argc, char **argv)
{
    ArgumentParser argument_parser;

    if (!argument_parser.run(argc, argv)) {
        return -1;
    }

    Context context;
    context.dataset_info = DatasetInfo(argument_parser.dataset_info_filename());
    create_directories(context.dataset_info, context.dataset_info.extraction_settings().class_name);
    exec_samples(context);

    // std::cout << "Extraction Settings: " << std::endl;
    // std::cout << dataset_info.extraction_settings().to_string() << std::endl;
    //
    // printf("Processing Positive Samples\n");
    // for (size_t i=0; i<positive_entries.size(); i++) {
    //
    //     std::cout << "Annotation Filename: " << positive_entries[i].annotation_filename << std::endl;
    //     context.annotations.clear();
    //     if (!positive_entries[i].annotation_filename.empty() && common::file_exists(positive_entries[i].annotation_filename)) {
    //         common::load_log_annotation(
    //             positive_entries[i].annotation_filename,
    //             positive_entries[i].annotation_name,
    //             context.annotations);
    //     }
    //
    //     printf("Processing log: %s\n", positive_entries[i].log_filename.c_str());
    //     context.name = positive_entries[i].name;
    //     context.hog_detector.reset_detection_stats();
    //     common::exec_samples_from_dataset_entry(positive_entries[i], sample_receiver_callback, &context);
    //     printf("\n");
    // }
    //

    // std::vector<std::vector<cv::Point> > annotations;
    // std::vector<base::samples::Sonar> samples;
    //
    // load_samples(dataset_info, samples, annotations);
    // create_directories(dataset_info, dataset_info.extraction_settings().class_name);
    //
    //
    // if (dataset_info.extraction_settings().extract_target_bbox) {
    //     extraction_bbox(samples, annotations, dataset_info, dataset_info.extraction_settings().class_name);
    // }
    // else if (dataset_info.extraction_settings().extract_target_orientations) {
    //     extraction_orientations(samples, annotations, dataset_info, dataset_info.extraction_settings().class_name);
    // }
    // else {
    //     extraction(samples, annotations, dataset_info, dataset_info.extraction_settings().class_name);
    // }

    return 0;
}
