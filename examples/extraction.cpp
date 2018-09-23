#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <boost/filesystem.hpp>
#include <base/samples/Sonar.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/SonarImagePreprocessing.hpp>
#include <sonar_processing/Preprocessing.hpp>
#include <sonarlog_target_tracking/ArgumentParser.hpp>
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
    int sample_index;
    DatasetInfo dataset_info;
    AnnotationList annotations;
    SonarHolder sonar_holder;
    SonarImagePreprocessing preprocessing;
    std::vector<std::string> files[kExtractionFoldersFinal];
    std::vector<cv::Rect> boxes;
    std::vector<cv::RotatedRect> rboxes;
    std::string log_basename;
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
    if (!create_directory(dataset_info.extraction_settings().extract_directory)){
        std::cout << "Failed to create the folder: " << dataset_info.extraction_settings().extract_directory << std::endl;
    }

    std::string dirpath;
    if (dataset_info.extraction_settings().extract_target_bbox)
        dirpath = dataset_info.extraction_settings().extract_directory;
    else if (dataset_info.extraction_settings().extract_target_orientations)
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "orientations");
    else if (dataset_info.extraction_settings().extract_rotation_norm)
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "rotated");
    else
        dirpath = file_join(dataset_info.extraction_settings().extract_directory, "original");

    if (!create_directory(dirpath)) {
        std::cout << "Failed to create the folder: " << dirpath << std::endl;
    }

    return file_join(dirpath, folder_type);
}

inline std::string get_folder(
    const DatasetInfo& dataset_info,
    const ExtractionFolders k,
    std::string folder_type = std::string(),
    std::string suffix = "")
{
    // if (k >= kExtractionFoldersFinal)
        // return get_output_directory(dataset_info, folder_type);
    // return file_join(get_output_directory(dataset_info, folder_type), FOLDERS[k]+suffix);
    return get_output_directory(dataset_info, folder_type);
}

inline void create_directories(const DatasetInfo& dataset_info, std::string folder_type = std::string())
{
    std::string output_directory = get_output_directory(dataset_info, folder_type);
    create_directory(output_directory);

    if (dataset_info.extraction_settings().save_source_image) {
        // create_directory(file_join(output_directory, FOLDERS[kSource]));
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

inline std::string get_filename(int index, std::string ext = ".png", std::string suffix = std::string(), std::string prefix = std::string())
{
    char buffer[256];
    snprintf(buffer, 256, "sample-%08d", index);

    std::stringstream ss;
    if (!prefix.empty()) ss << prefix;
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

inline void write_bbox_annotation(
    const std::string& filepath,
    int id,
    const cv::Rect& rect,
    const cv::Point2f& center,
    const cv::Size2f& size,
    float angle)
{
    float x1 = rect.x;
    float y1 = rect.y;
    float x2 = rect.width+x1;
    float y2 = rect.height+y1;
    char buffer[1024];

    snprintf(buffer, 1024, "%d %f %f %f %f %f %f %f %f %f\n",
        id,
        x1, y1, x2, y2,
        center.x, center.y, size.width, size.height,
        angle);

    printf("annotation: %s\n", buffer);
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
    // write_bbox_annotation(filepath, id, center, size, rbbox.angle);
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

    // if (bbox.angle < 0) {
    //     bbox.angle = bbox.angle+180;
    // }

    return bbox;

}

inline cv::RotatedRect load_rotated_rect_from_mask(const cv::Mat& mask)
{
    std::vector<cv::Point> contour = preprocessing::find_biggest_contour(mask);
    return  load_rotated_rect(contour);
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

void save_bbox_data(
    const cv::Mat image,
    const cv::Mat& gt_mask,
    const cv::Rect& rect,
    const cv::RotatedRect& rbox,
    const size_t index,
    const DatasetInfo& dataset_info,
    const std::string& root_dir,
    std::vector<std::string>& files,
    std::string suffix = std::string(),
    std::string prefix = std::string())
{
    std::string image_filepath = file_join(root_dir, get_filename(index, ".png", suffix, prefix));
    files.push_back(image_filepath);
    write_image(image_filepath, image);

    std::string gt_mask_filepath = file_join(root_dir, get_filename(index, "-mask.png", suffix, prefix));
    write_image(gt_mask_filepath, gt_mask);

    write_bbox_annotation(
        file_join(root_dir, get_filename(index, ".txt", suffix, prefix)),
        dataset_info.extraction_settings().class_id,
        rect,
        cv::Point2f(rbox.center.x-rect.x, rbox.center.y-rect.y),
        rbox.size,
        rbox.angle);
}

void save_images_bbox_data(
    const cv::Mat images[kExtractionFoldersFinal],
    const cv::Mat& gt_mask,
    const cv::Rect& rect,
    const cv::RotatedRect& rbox,
    const bool save_image[kExtractionFoldersFinal],
    const size_t index,
    const DatasetInfo& dataset_info,
    std::vector<std::string> (&files)[kExtractionFoldersFinal],
    std::string folder_type = std::string(),
    std::string suffix = std::string(),
    std::string prefix = std::string())
{
    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (save_image[k]) {
            std::string root_dir = get_folder(dataset_info, (ExtractionFolders)k, folder_type);
            save_bbox_data(images[k], gt_mask, rect, rbox, index, dataset_info, root_dir, files[k], suffix, prefix);
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
    const float theta_step,
    std::vector<std::string> (&files)[kExtractionFoldersFinal],
    std::string folder_type = std::string())
{
    const float gt_angle = rbox.angle;

    cv::Mat rot_images[kExtractionFoldersFinal];

    for (float theta = 0; theta <= 180; theta +=theta_step) {
        float theta_rot = gt_angle-theta;
        cv::Mat rot_gt_mask;
        rotate(gt_mask, rot_gt_mask, theta_rot);
        rotate_images(images, rot_images, save_image, theta_rot);

        // load ground truth rotated rect
        cv::RotatedRect rot_rbox = load_rotated_rect_from_mask(rot_gt_mask);

        // load bounding box
        cv::Rect rot_rect =  image_util::bounding_rect(rot_gt_mask);

        char buffer[256];
        snprintf(buffer, 256, "-%04d-bbox", (int)theta);

        save_images_bbox_data(
            rot_images, rot_gt_mask, rot_rect, rot_rbox, save_image,
            index, dataset_info, files, folder_type, buffer);
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


void draw_text(cv::Mat& dst, cv::Point text_org, std::string text) {
    const double font_scale = 0.8;
    const int font_face = cv::FONT_HERSHEY_COMPLEX;
    const int text_thickness = 2;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, text_thickness, &baseline);
    cv::putText(dst, text.c_str(), text_org, font_face, font_scale, cv::Scalar(0, 0, 255), text_thickness);
}
void extract_sample_bbox(
    Context& context,
    const cv::Mat& source_image,
    const cv::Mat& source_mask,
    const std::vector<cv::Point>& annotation)
{
    std::string class_name = context.dataset_info.extraction_settings().class_name;
    int class_id = context.dataset_info.extraction_settings().class_id;

    cv::Mat gt_mask;
    image_util::draw_mask_from_points(source_image.size(), annotation, gt_mask);

    cv::Mat images[kExtractionFoldersFinal];
    apply_preprocessing(source_image, source_mask, context.preprocessing, images);

    bool save_options[kExtractionFoldersFinal];
    load_save_options(context.dataset_info.extraction_settings(), save_options);

    cv::Rect bbox = image_util::bounding_rect(gt_mask);
    cv::RotatedRect rbbox = load_rotated_rect(annotation);

    std::string folder_type = file_join(class_name, context.log_basename);

    cv::Mat canvas;
    cv::cvtColor(source_image, canvas, cv::COLOR_GRAY2BGR);
    cv::rectangle(canvas, bbox, cv::Scalar(0, 255, 0), 3);
    image_util::draw_rotated_rect(canvas, cv::Scalar(255, 0, 0), rbbox);

    cv::Point2f pts[4];
    rbbox.points( pts );
    char buff[64];
    snprintf(buff,64, "%d", (int)rbbox.angle);
    draw_text(canvas, cv::Point(pts[0].x, pts[0].y), buff);

    cv::imshow("canvas", canvas);
    cv::waitKey(15);

    save_images_bbox_data(
        images,
        gt_mask,
        bbox,
        rbbox,
        save_options,
        // context.sample_count,
        context.sample_index,
        context.dataset_info,
        context.files,
        folder_type);

    cv::Mat flip_source_image;
    cv::flip(source_image, flip_source_image, 1);

    cv::Mat flip_gt_mask;
    cv::flip(gt_mask, flip_gt_mask, 1);

    cv::Rect flip_bbox = image_util::bounding_rect(flip_gt_mask);
    cv::RotatedRect flip_rbbox = load_rotated_rect_from_mask(flip_gt_mask);

    cv::Mat flip_canvas;
    cv::cvtColor(flip_source_image, flip_canvas, cv::COLOR_GRAY2BGR);
    cv::rectangle(flip_canvas, flip_bbox, cv::Scalar(0, 255, 0), 3);
    image_util::draw_rotated_rect(flip_canvas, cv::Scalar(255, 0, 0), flip_rbbox);

    flip_rbbox.points(pts);
    snprintf(buff,64, "%d", (int)flip_rbbox.angle);
    draw_text(flip_canvas, cv::Point(pts[0].x, pts[0].y), buff);

    cv::imshow("flip_canvas", flip_canvas);
    cv::waitKey(15);

    cv::Mat flip_images[kExtractionFoldersFinal];
    for (size_t k = 0; k < kExtractionFoldersFinal; k++) {
        if (!save_options[k]) continue;
        cv::flip(images[k], flip_images[k], 1);
    }

    save_images_bbox_data(
        flip_images,
        flip_gt_mask,
        flip_bbox,
        flip_rbbox,
        save_options,
        context.sample_index,
        context.dataset_info,
        context.files,
        folder_type,
        std::string(),
        "flip-");

    if (context.dataset_info.extraction_settings().extract_target_orientations) {
        save_bbox_data_multiple_angles(
            images,
            gt_mask,
            bbox,
            rbbox,
            save_options,
            context.sample_count,
            context.dataset_info,
            context.dataset_info.extraction_settings().extract_target_orientations_step,
            context.files,
            class_name);
    }
}

void sample_receiver_callback(const base::samples::Sonar& sample, int sample_index, void *user_data)
{
    printf("Current sample: %d\n", sample_index);
    Context *pContext = reinterpret_cast<Context*>(user_data);
    pContext->sample_index = sample_index;

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

    if (pContext->dataset_info.extraction_settings().extract_target_bbox) {
        extract_sample_bbox(*pContext, source_image, source_mask, annotation);
    }
    else {
        extract_sample(*pContext, source_image, source_mask, annotation);
        if (pContext->dataset_info.extraction_settings().extract_target_orientations)
            extract_sample_multiple_angles(*pContext, source_image, source_mask, annotation);
    }

    pContext->sample_count++;
}

void load_log_annotation(Context& context, sonarlog_target_tracking::DatasetInfoEntry& entry)
{
    context.annotations.clear();
    if (!entry.annotation_filename.empty() && common::file_exists(entry.annotation_filename)) {
        common::load_log_annotation(entry.annotation_filename, entry.annotation_name, context.annotations);
    }
}

void save_annotation_files(Context& context)
{
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
            context.log_basename = boost::filesystem::basename(entries[i].log_filename);
            std::string folder_type = file_join(context.dataset_info.extraction_settings().class_name, context.log_basename);
            std::string output_folder = get_folder(context.dataset_info, kExtractionFoldersFinal, folder_type);
            if (!create_directory(output_folder))
                std::cout << "Failed to create the folder " << output_folder << std::endl;
            common::exec_training_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
        }
    }
    else {
        for (size_t i=0; i<entries.size(); i++) {
            load_log_annotation(context, entries[i]);
            common::exec_samples_from_dataset_entry(entries[i], sample_receiver_callback, &context);
        }
    }

    if (!context.dataset_info.extraction_settings().extract_target_bbox)
        save_annotation_files(context);
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

    return 0;
}
