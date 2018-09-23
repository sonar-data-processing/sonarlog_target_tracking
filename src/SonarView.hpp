#ifndef _SonarView_hpp_
#define _SonarView_hpp_

#include <iostream>
#include <ctime>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <base/Angle.hpp>
#include <sonar_processing/SonarHolder.hpp>
#include <sonar_processing/ImageUtil.hpp>

#define SCALE_FACTOR     0.5
#define MIN_SEL_AREA     8
#define MAX_SEL_AREA     256
#define IMAGE_TITLE      "cartesian_image"
#define SEL_WINDOW_SIZE  256
#define SEL_WINDOW_TITLE "selection_image"

#define KEY_ZOOM_IN1  61  //zoom in
#define KEY_ZOOM_IN0  43  //zoom in
#define KEY_ZOOM_IN2  171 //zoom in
#define KEY_ZOOM_OUT0 45  // zoom out
#define KEY_ZOOM_OUT1 95  // zoom out
#define KEY_ZOOM_OUT2 173 // zoom out
#define KEY_LEFT      81  // left
#define KEY_UP        82  // up
#define KEY_RIGHT     83  // right
#define KEY_DOWN      84  // down

static const uchar KEYS[] = {
    KEY_ZOOM_IN0,
    KEY_ZOOM_IN1,
    KEY_ZOOM_IN2,
    KEY_ZOOM_OUT0,
    KEY_ZOOM_OUT1,
    KEY_ZOOM_OUT2,
    KEY_LEFT,
    KEY_UP,
    KEY_RIGHT,
    KEY_DOWN
};

struct PointComparator {
    virtual bool point_validate(cv::Point2f pt0, cv::Point2f pt1) = 0;
    virtual float point_distance(cv::Point2f pt0, cv::Point2f pt1) = 0;
};

struct HorizontalComparator : PointComparator {
    virtual float point_distance(cv::Point2f pt0, cv::Point2f pt1) { return fabs(pt0.y - pt1.y); }
};

struct LeftComparator : HorizontalComparator {
    virtual bool point_validate(cv::Point2f pt1, cv::Point2f pt2) { return pt1.x < pt2.x; }
} left_comparator;

struct RightComparator : HorizontalComparator {
    virtual bool point_validate(cv::Point2f pt1, cv::Point2f pt2) { return pt1.x > pt2.x; }
} right_comparator;

struct VerticalComparator : PointComparator {
    virtual float point_distance(cv::Point2f pt0, cv::Point2f pt1) { return fabs(pt0.x - pt1.x); }
};

struct UpComparator : VerticalComparator {
    virtual bool point_validate(cv::Point2f pt1, cv::Point2f pt2) { return pt1.y < pt2.y; }
} up_comparator;

struct DownComparator : VerticalComparator {
    virtual bool point_validate(cv::Point2f pt1, cv::Point2f pt2) { return pt1.y > pt2.y; }
} down_comparator;

enum SCANNING_DIRECTION {
    SCANNING_HORIZONTAL = 1,
    SCANNING_VERTICAL = 2
};

struct SonarViewContext {
    SonarViewContext(sonar_processing::SonarHolder _sonar_holder)
        : sel_point0(-1, -1)
        , sel_point1(-1, -1)
        , exit_selection_window(false)
        , sel_polar_index(-1)
    {
        sonar_holder = _sonar_holder;
    }

    cv::Rect sel_rect;
    cv::Rect transform_rect;

    float sel_scale_factor;
    float sel_rect_scale_factor;
    cv::Point2f sel_rect_translation;

    cv::Point sel_point0;
    cv::Point sel_point1;
    cv::Point2f sel_origin;

    cv::Mat scale_image;
    cv::Mat canvas;
    cv::Mat sel_image;
    cv::Mat sel_canvas;

    int sel_polar_index;
    cv::Point2f sel_cart_point;

    std::vector<int> sel_right_indices;
    std::vector<int> sel_line_indices;
    std::vector<int> sel_column_indices;

    bool exit_selection_window;

    sonar_processing::SonarHolder sonar_holder;

    void update_selection_rectangle() {
        sel_rect.x = floor(std::min<int>(sel_point0.x, sel_point1.x) / SCALE_FACTOR);
        sel_rect.y = floor(std::min<int>(sel_point0.y, sel_point1.y) / SCALE_FACTOR);
        sel_rect.width = ceil(abs(sel_point0.x-sel_point1.x) / SCALE_FACTOR);
        sel_rect.height = ceil(abs(sel_point0.y-sel_point1.y) / SCALE_FACTOR);
        transform_rect = sel_rect;
    }

    bool is_transform_selection_rectangle(int trans_x, int trans_y, float scale_factor) {
        if ( scale_factor != sel_rect_scale_factor ||
             trans_x != sel_rect_translation.x &&
             sel_rect.x+trans_x >= 0 &&
             sel_rect.x+trans_x < sonar_holder.cart_size().width ||
             trans_y != sel_rect_translation.y &&
             sel_rect.y+trans_y >= 0 &&
             sel_rect.y+trans_y < sonar_holder.cart_size().height ){

            return true;
        }

        return false;
    }

    cv::Rect transform_selection_rectangle(int trans_x, int trans_y, float scale_factor) {
        int new_x = sel_rect.x + trans_x;
        int new_y = sel_rect.y + trans_y;

        cv::Point2f center_point = cv::Point2f(new_x+sel_rect.width/2, new_y+sel_rect.height/2);

        cv::Point2f top_left_point = cv::Point(new_x, new_y);
        cv::Point2f bottom_right_point = cv::Point(new_x+sel_rect.width, new_y+sel_rect.height);

        cv::Point2f transform_top_left_point = top_left_point - center_point;
        transform_top_left_point *= scale_factor;
        transform_top_left_point += center_point;

        cv::Point2f transform_bottom_right_point = bottom_right_point - center_point;
        transform_bottom_right_point *= scale_factor;
        transform_bottom_right_point += center_point;

        return cv::Rect(transform_top_left_point, transform_bottom_right_point);
    }

    bool update_and_validate_selection_final_point(int x, int y) {
        int w = sonar_holder.cart_size().width * SCALE_FACTOR;
        int h = sonar_holder.cart_size().height * SCALE_FACTOR;
        int xx  = (x < 0) ? 0 : (x > w-1) ? w-1 : x;
        int yy  = (y < 0) ? 0 : (y > h-1) ? h-1 : y;
        int dx = xx - sel_point0.x;
        int dy = yy - sel_point0.y;
        sel_point1.x = (abs(dx) < MAX_SEL_AREA) ? xx : sel_point0.x + MAX_SEL_AREA * ((dx > 0) ? 1 : -1);
        sel_point1.y = (abs(dy) < MAX_SEL_AREA) ? yy : sel_point0.y + MAX_SEL_AREA * ((dy > 0) ? 1 : -1);
        return (abs(dx) >= MIN_SEL_AREA  && abs(dy) >= MIN_SEL_AREA);
    }

    void draw_selection_rectangle() {
        draw_selection_rectangle(sel_point0, sel_point1);
    }

    void draw_selection_rectangle(cv::Point2f pt0, cv::Point2f pt1) {
        cv::cvtColor(scale_image, canvas, CV_GRAY2BGR);
        cv::rectangle(canvas, pt0, pt1, cv::Scalar(0, 0, 255), 2, CV_AA);
        cv::imshow(IMAGE_TITLE, canvas);
    }

    cv::Point2f scale_to_selection(cv::Point2f pt) {
        cv::Point2f ret = pt;
        ret -= sel_origin;
        ret *= sel_scale_factor;
        return ret;
    }

    cv::Rect_<float> scale_to_selection(cv::Rect_<float> rc) {
        return cv::Rect_<float>(scale_to_selection(rc.tl()), scale_to_selection(rc.br()));
    }

    std::vector<cv::Point2f> scale_to_selection(std::vector<cv::Point2f> points) {
        std::vector<cv::Point2f> scaled_points(points.size());
        for (size_t i = 0; i < scaled_points.size(); i++) {
            scaled_points[i] = scale_to_selection(points[i]);
        }
        return scaled_points;
    }

    float sel_approx_beam_size() {
        int dy = sonar_holder.cart_origin().y - transform_rect.br().y;
        int dx = sonar_holder.cart_origin().x - transform_rect.br().x;
        int radius = sqrt(dx * dx + dy * dy);
        return (cos(sonar_holder.beam_width() - M_PI_2) * radius * 2) / sonar_holder.beam_count() ;
    }

    void find_min_distance_points(const cv::Point& point, const std::vector<cv::Point2f>& points, std::vector<int>& indices, int number_of_points) {
        std::vector<float> min_dist_values(number_of_points, FLT_MAX);
        indices.assign(number_of_points, -1);

        for (int idx = 0; idx < points.size(); idx++) {
            float dx = point.x - points[idx].x;
            float dy = point.y - points[idx].y;
            float dist = sqrt(dx * dx + dy * dy);

            for (size_t k = 0; k < min_dist_values.size(); k++) {
                if (dist < min_dist_values[k]) {
                    int current_idx = idx;

                    float aux_dist = min_dist_values[k];
                    min_dist_values[k] = dist;

                    int aux_idx = indices[k];
                    indices[k] = current_idx;

                    for (size_t l = k+1; l < min_dist_values.size(); l++) {
                        dist = aux_dist;
                        aux_dist = min_dist_values[l];
                        min_dist_values[l] = dist;

                        current_idx = aux_idx;
                        aux_idx = indices[l];
                        indices[l] = current_idx;
                    }

                    break;
                }
            }
        }
    }

    void cart_to_polar(int x, int y, int& bin_out, int& beam_out) {

        bin_out = 0;
        beam_out = 0;

        cv::Point2f point(x, y);
        std::vector<cv::Point2f> cart_points = sonar_holder.cart_center_points();

        std::vector<int> indices;
        find_min_distance_points(point, cart_points, indices, 4);
        bin_out = sonar_holder.index_to_bin(indices[0]);
        beam_out = sonar_holder.index_to_beam(indices[0]);
    }

    // void compute_neighbors_direction(int polar_index, std::vector<int>& direction_mapping, std::vector<int>& neighbors_indices, std::vector<float>& neighbors_angles) {
    //     const int kNumberOfNeighbors = 4;

    //     sonar_holder.GetNeighborhoodAngles(polar_index, polar_index, neighbors_indices, neighbors_angles);
    //     direction_mapping.assign(kNumberOfNeighbors, -1);

    //     float direction_angles[4] = {
    //         0,              // 0deg
    //         M_PI,           // 180deg
    //         M_PI_2,         // 90deg
    //         M_PI + M_PI_2   // 270deg
    //     };

    //     for (size_t k = 0; k < kNumberOfNeighbors; k++) {
    //         direction_mapping[k] = sonar_holder.GetMinAngleDistance(neighbors_angles, neighbors_indices, direction_angles[k]);
    //     }
    // }

    // int min_angle_difference_element(const std::vector<int>& neighbors_indices,
    //                                  const std::vector<float>& neighbors_angles,
    //                                  float angle) {
    //     float min_theta = FLT_MAX;
    //     int min_idx = -1;
    //     for (size_t i = 0; i < neighbors_indices.size(); i++) {
    //         if (neighbors_indices[i] != -1) {
    //             float theta = base::MathUtil::angle_difference(neighbors_angles[i], angle);
    //             if (theta < min_theta) {
    //                 min_theta = theta;
    //                 min_idx = i;
    //             }
    //         }
    //     }
    //     return min_idx;

    // }

    void recursive_find_cart_right_bins(int ref_polar_index, int polar_index,
                                        const std::vector<cv::Point2f>& cart_points,
                                        std::vector<int>& polar_indices) {
        if (polar_index == -1) return;

        if (sonar_holder.index_to_beam(polar_index) >= sonar_holder.beam_count()-2 ||
            sonar_holder.index_to_bin(polar_index) >= sonar_holder.bin_count()-2) {
            return;
        }

        polar_indices.push_back(polar_index);

        std::vector<int> neighbors_indices;
        sonar_holder.GetNeighborhood(polar_index, neighbors_indices);

        cv::Point2f ref_pt = cart_points[ref_polar_index];
        cv::Point2f pt = cart_points[polar_index];

        int min_index = -1;
        float min_dy = FLT_MAX;

        for (size_t i = 0; i < neighbors_indices.size(); i++) {
            cv::Point2f neighbor_pt = cart_points[neighbors_indices[i]];

            if (neighbor_pt.x > pt.x) {
                float dy = ref_pt.y - neighbor_pt.y;

                if (fabs(dy) < min_dy) {
                    min_dy = fabs(dy);
                    min_index = i;
                }
            }
        }

        recursive_find_cart_right_bins(ref_polar_index, neighbors_indices[min_index], cart_points, polar_indices);
    }

    void cart_right_bins(int polar_index, std::vector<int>& polar_indices) {
        recursive_find_cart_right_bins(polar_index, polar_index, sonar_holder.cart_center_points(), polar_indices);
    }

    void get_cart_left_bins(int polar_index, std::vector<int>& indices, int max_size = -1) {
        get_cart_bins(polar_index, indices, &left_comparator, max_size);
    }

    void get_cart_right_bins(int polar_index, std::vector<int>& indices, int max_size = -1) {
        get_cart_bins(polar_index, indices, &right_comparator, max_size);
    }

    void get_cart_up_bins(int polar_index, std::vector<int>& indices, int max_size = -1, cv::Point2f ref_point = cv::Point2f(-1, -1)) {
        get_cart_bins(polar_index, indices, &up_comparator, max_size, ref_point);
    }

    void get_cart_down_bins(int polar_index, std::vector<int>& indices, int max_size = -1, cv::Point2f ref_point = cv::Point2f(-1, -1)) {
        get_cart_bins(polar_index, indices, &down_comparator, max_size, ref_point);
    }

    void get_sonar_cart_line(int polar_index, std::vector<int>& indices, int line_size = -1) {
        int line_size_2 = (line_size == -1) ? -1 : line_size / 2;
        get_cart_left_bins(polar_index, indices, line_size_2);
        std::reverse(indices.begin(), indices.end());
        indices.push_back(polar_index);
        get_cart_right_bins(polar_index, indices, line_size_2);
    }

    void get_sonar_cart_column(int polar_index, std::vector<int>& indices, int column_size = -1, cv::Point2f ref_point = cv::Point2f(-1, -1)) {
        int column_size_2 = (column_size == -1) ? -1 : column_size / 2;
        get_cart_up_bins(polar_index, indices, column_size_2, ref_point);
        std::reverse(indices.begin(), indices.end());
        indices.push_back(polar_index);
        get_cart_down_bins(polar_index, indices, column_size_2, ref_point);
    }

    int min_distance_index(cv::Point2f origin_point, cv::Point2f current_cart_point,
                           const std::vector<cv::Point2f>& cart_points,
                           const std::vector<int>& indices, PointComparator *cmp) {
        int min_index = -1;
        float min_dist = FLT_MAX;
        float neighbor_dist = 0;

        for (size_t i = 0; i < indices.size(); i++) {
            cv::Point2f point = cart_points[indices[i]];
            if (cmp->point_validate(point, current_cart_point)) {
                if ((neighbor_dist = cmp->point_distance(origin_point, point)) < min_dist) {
                    min_dist = neighbor_dist;
                    min_index = i;
                }
            }
        }

        return min_index;
    }

    bool is_cart_bins_scanning_complete(int polar_index, int count, int max_count = -1) {
        int beam = sonar_holder.index_to_beam(polar_index);
        int bin = sonar_holder.index_to_bin(polar_index);

        if (beam <= 0 || beam >= sonar_holder.beam_count()-2 ||
            bin <= 0 || bin >= sonar_holder.bin_count()-2 ||
            (max_count != -1 && count >= max_count) ||
            polar_index == -1) {
            return true;
        }

        return false;
    }

    int next_cart_index(int polar_index,
                        cv::Point2f origin_point,
                        const std::vector<cv::Point2f>& cart_points,
                        PointComparator *cmp) {
        std::vector<int> neighbors_indices;
        sonar_holder.GetNeighborhood(polar_index, neighbors_indices);
        int min_index = min_distance_index(origin_point, cart_points[polar_index], cart_points, neighbors_indices, cmp);
        return neighbors_indices[min_index];
    }

    void get_cart_bins(int polar_index,
                       std::vector<int>& indices,
                       PointComparator *cmp,
                       int max_count = -1,
                       cv::Point2f ref_point = cv::Point2f(-1, -1)) {

        std::vector<cv::Point2f> cart_center_points = sonar_holder.cart_center_points();

        int count = 0;
        int current_index = polar_index;

        cv::Point2f origin_point;

        if (ref_point.x != -1 && ref_point.y != -1) {
            origin_point = ref_point;
        }
        else {
            origin_point = cart_center_points[polar_index];
        }

        current_index = next_cart_index(current_index, origin_point, cart_center_points, cmp);
        while (!is_cart_bins_scanning_complete(current_index, count, max_count)) {
            indices.push_back(current_index);
            current_index = next_cart_index(current_index, origin_point, cart_center_points, cmp);
            count++;
        }

    }

    void compute_right_bins(int polar_index, std::vector<int>& indices) {
        std::vector<int> neighbors_indices;

        int current_index = polar_index;

        std::vector<cv::Point2f> cart_center_points = sonar_holder.cart_center_points();
        cv::Point2f ref_pt = cart_center_points[polar_index];

        do {

            int beam = sonar_holder.index_to_beam(current_index);
            int bin = sonar_holder.index_to_bin(current_index);

            if (beam <= 0 || beam >= sonar_holder.beam_count()-2 ||
                bin <= 0 || bin >= sonar_holder.bin_count()-2) {
                break;
            }

            indices.push_back(current_index);
            sonar_holder.GetNeighborhood(current_index, neighbors_indices);

            int min_index = -1;
            float min_dy = FLT_MAX;
            cv::Point2f pt = cart_center_points[current_index];
            for (size_t i = 0; i < neighbors_indices.size(); i++) {
                cv::Point2f neighbor_pt = cart_center_points[neighbors_indices[i]];
                if (neighbor_pt.x > pt.x) {
                    float dy = ref_pt.y - neighbor_pt.y;
                    if (fabs(dy) < min_dy) {
                        min_dy = fabs(dy);
                        min_index = i;
                    }
                }
            }

            current_index = neighbors_indices[min_index];
        } while (current_index != -1);
    }
};


void SonarView_draw_arrowed_line(cv::Mat img, cv::Point2f pt1, cv::Point2f pt2, const cv::Scalar& color, int thickness = 1) {
    const double cos = 0.866;
    const double sin = 0.500;

    int n = std::max(abs(pt1.x - pt2.x), abs(pt1.y - pt2.y));
    float dx = (pt1.x - pt2.x) / n * 10;
    float dy = (pt1.y - pt2.y) / n * 10;

    cv::Point2f tip_pt1 = cv::Point2f(pt2.x + (dx * cos + dy * -sin), pt2.y + (dx * sin + dy * cos));
    cv::Point2f tip_pt2 = cv::Point2f(pt2.x + (dx * cos + dy * sin), pt2.y + (dx * -sin + dy * cos));
    cv::line(img, pt1, pt2, color, thickness);
    cv::line(img, pt2, tip_pt1, color, thickness);
    cv::line(img, pt2, tip_pt2, color, thickness);
}

// void SonarView_draw_angles(SonarViewContext& context, int polar_index) {
//     std::vector<int> neighbors_indices;
//     std::vector<float> neighbors_angles;

//     int sel_polar_index = (context.sel_polar_index != -1) ? context.sel_polar_index : polar_index;
//     context.sonar_holder.GetNeighborhoodAngles(sel_polar_index, polar_index, neighbors_indices, neighbors_angles);

//     int element_mask[] = {1, 1, 1,
//                           1, 0, 1,
//                           1, 1, 1};

//     std::vector<cv::Point2f> pts;
//     std::vector<uint32_t> indices;
//     std::vector<cv::Point2f> cart_center_points = context.sonar_holder.cart_center_points();
//     for (size_t i = 0; i < neighbors_indices.size(); i++) {

//         if (element_mask[i]) {
//             int polar_index = neighbors_indices[i];
//             cv::Point2f pt = cart_center_points[polar_index];
//             pt= context.scale_to_selection(pt);
//             cv::circle(context.sel_canvas, pt, 3, cv::Scalar(0, 0, 255), 2);
//             pts.push_back(pt);
//             indices.push_back(i);
//         }
//     }

//     char buff[256];
//     for (size_t i = 0; i < indices.size(); i++){
//         sprintf(buff, "%0.2f", base::Angle::rad2Deg(neighbors_angles[indices[i]]));
//         cv::putText(context.sel_canvas, buff, pts[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255));
//     }
// }

void SonarView_draw_sonar_cart_path(SonarViewContext& context, int polar_index, const std::vector<int>& indices, int direction = 0) {
    if (indices.size() > 1) {
        std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();

        cv::Point2f pt = cart_points[polar_index];
        pt= context.scale_to_selection(pt);

        for (size_t i = 1; i < indices.size(); i++) {
            int polar_index0 = indices[i-1];
            int polar_index1 = indices[i];

            int bin = context.sonar_holder.index_to_bin(polar_index0);
            int beam = context.sonar_holder.index_to_beam(polar_index0);

            cv::Point2f pt0 = cart_points[polar_index0];
            cv::Point2f pt1 = cart_points[polar_index1];
            cv::line(context.canvas, pt0 * SCALE_FACTOR, pt1 * SCALE_FACTOR, cv::Scalar(0, 255, 0), 2);

            pt0 = context.scale_to_selection(pt0);
            pt1 = context.scale_to_selection(pt1);

            if (direction == 0) {
                cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt0.x, pt.y), cv::Scalar(255, 255, 0), 1);
                cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2);
                cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt.y), cv::Point2f(pt1.x, pt.y), cv::Scalar(0, 255, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), 2, cv::Scalar(0, 0, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt1.x, pt1.y), 2, cv::Scalar(0, 0, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt0.x, pt.y), 1, cv::Scalar(255, 0, 0), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt1.x, pt.y), 1, cv::Scalar(255, 0, 0), 2);
            }
            else if (direction == 1) {
                cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt.x, pt0.y), cv::Scalar(255, 255, 0), 1);
                cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2);
                cv::line(context.sel_canvas, cv::Point2f(pt.x, pt0.y), cv::Point2f(pt.x, pt1.y), cv::Scalar(0, 255, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), 2, cv::Scalar(0, 0, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt1.x, pt1.y), 2, cv::Scalar(0, 0, 255), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt.x, pt0.y), 1, cv::Scalar(255, 0, 0), 2);
                cv::circle(context.sel_canvas, cv::Point2f(pt.x, pt1.y), 1, cv::Scalar(255, 0, 0), 2);
            }
        }
    }
}

void SonarView_draw_sonar_cart_line(SonarViewContext& context, int polar_index, const std::vector<int>& indices) {
    SonarView_draw_sonar_cart_path(context, polar_index, indices, 0);
}

void SonarView_draw_sonar_cart_column(SonarViewContext& context, int polar_index, const std::vector<int>& indices) {
    SonarView_draw_sonar_cart_path(context, polar_index, indices, 1);
}

void SonarView_initialize_line_indices(SonarViewContext& context) {
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
    context.sel_line_indices.clear();
    context.sel_column_indices.clear();
    // sonar_processing::basic_operations::line_indices(context.sonar_holder, context.sel_polar_index, context.sel_line_indices);
    // sonar_processing::basic_operations::column_indices(context.sonar_holder, context.sel_polar_index, context.sel_column_indices);
}

void SonarView_draw_cart_path(SonarViewContext& context) {
    SonarView_initialize_line_indices(context);
    SonarView_draw_sonar_cart_line(context, context.sel_polar_index, context.sel_line_indices);
    SonarView_draw_sonar_cart_column(context, context.sel_polar_index, context.sel_column_indices);
    cv::circle(context.sel_canvas, cv::Point(context.sel_cart_point.x, context.sel_cart_point.y), 3, cv::Scalar(0, 255, 0), 2);
    cv::imshow(SEL_WINDOW_TITLE, context.sel_canvas);
    cv::imshow(IMAGE_TITLE, context.canvas);
}

void SonarView_draw_cart_path2(SonarViewContext& context) {
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
    std::vector<float> angles;
    context.sel_right_indices.clear();
    context.cart_right_bins(context.sel_polar_index, context.sel_right_indices);
    SonarView_draw_sonar_cart_line(context, context.sel_polar_index, context.sel_right_indices);
    cv::imshow(SEL_WINDOW_TITLE, context.sel_canvas);
    cv::imshow(IMAGE_TITLE, context.canvas);
}

void SonarView_selection_image_left_mouse_button_up(SonarViewContext& context, int x, int y) {
    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;
    context.cart_to_polar(click_point.x, click_point.y, bin, beam);

    context.sel_polar_index = beam * context.sonar_holder.bin_count() + bin;
    context.sel_cart_point = context.sonar_holder.cart_center_point(bin, beam);
    context.sel_cart_point = context.scale_to_selection(context.sel_cart_point);

    SonarView_draw_cart_path(context);
}

std::vector<cv::Point2f> SonarView_get_column_intercept_points(SonarViewContext& context, std::vector<int> column_indices, cv::Point2f ref_point) {

    std::vector<cv::Point2f> intercept_points;
    intercept_points.assign(column_indices.size(), cv::Point2f(-1, -1));
    for (size_t i = 0; i < column_indices.size(); i++) {
        int index = column_indices[i];

        cv::Point2f origin = context.sonar_holder.cart_origin();
        origin = context.scale_to_selection(origin);
        std::vector<float> bearings = context.sonar_holder.bearings();

        std::vector<cv::Point2f> points = context.sonar_holder.GetSectorPoints(index);

        points = context.scale_to_selection(points);
        cv::Rect_<float> rc = sonar_processing::image_util::bounding_rect(cv::Mat(points));

        int bin = context.sonar_holder.index_to_bin(index);
        int beam = context.sonar_holder.index_to_beam(index);

        float r0 = (bin+0) * context.sel_scale_factor;
        float r1 = (bin+1) * context.sel_scale_factor;
        float r2 = r0 + (r1 - r0) / 2;
        float t0 = bearings[beam+0];
        float t1 = bearings[beam+1];

        for (int y = rc.tl().y; y < rc.br().y+1; y++) {
            cv::Point2f pt = cv::Point2f(ref_point.x, y);

            float dx = origin.x - pt.x;
            float dy = origin.y - pt.y;
            float r = sqrt(dx * dx + dy * dy);
            float t = atan2(dy, dx) - M_PI_2;

            if (r >= r2 && r <= r2+1 && t >= t0 && t <= t1) {
                intercept_points[i] = pt;
                break;
            }
        }
    }

    return intercept_points;
}

std::vector<cv::Point2f> SonarView_get_lines_intercept_points(SonarViewContext& context, std::vector<int> line_indices, cv::Point2f ref_point) {
    std::vector<cv::Point2f> intercept_points;

    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();
    std::vector<float> bearings = context.sonar_holder.bearings();

    cv::Point2f origin = context.sonar_holder.cart_origin();
    origin = context.scale_to_selection(origin);

    for (size_t i = 0; i < line_indices.size(); i++) {
        int index = line_indices[i];

        std::vector<cv::Point2f> points = context.sonar_holder.GetSectorPoints(index);
        points = context.scale_to_selection(points);

        cv::Rect_<float> rc = sonar_processing::image_util::bounding_rect(cv::Mat(points));

        int bin = context.sonar_holder.index_to_bin(index);
        int beam = context.sonar_holder.index_to_beam(index);

        float r0 = (bin+0) * context.sel_scale_factor;
        float r1 = (bin+1) * context.sel_scale_factor;
        float t0 = bearings[beam+0];
        float t1 = bearings[beam+1];

        bool within_sector = false;

        cv::Point2f pt0 = cv::Point2f(-1, -1);
        cv::Point2f pt1 = cv::Point2f(-1, -1);

        for (int x = rc.tl().x; x <= rc.br().x+1; x++) {
            cv::Point2f pt = cv::Point2f(x, ref_point.y);

            float dx = origin.x - pt.x;
            float dy = origin.y - pt.y;
            float r = sqrt(dx * dx + dy * dy);
            float t = atan2(dy, dx) - M_PI_2;

            if (r >= r0 && r <= r1 && t >= t0 && t <= t1) {
                if (pt0.x == -1 && pt0.y == -1) pt0 = pt;
                within_sector = true;
            }
            else {
                if (within_sector) pt1 = pt;
                within_sector = false;
            }
        }

        intercept_points.push_back(pt0);
        if (i == line_indices.size()-1) {
            intercept_points.push_back(pt1);
        }
    }

    std::vector<cv::Point2f> mid_points;
    for (size_t k = 1; k < intercept_points.size(); k++) {
        cv::Point2f pt0 = intercept_points[k-1];
        cv::Point2f pt1 = intercept_points[k];
        cv::Point2f pt = cv::Point2f(pt0.x + (pt1.x - pt0.x) / 2, pt0.y);
        mid_points.push_back(pt);
    }

    return mid_points;
}

std::vector<std::vector<int> > SonarView_get_line_neighbors(SonarViewContext& context, std::vector<int> line_indices, int neighbor_size) {
    std::vector<std::vector<int> > columns_indices;

    for (size_t i = 0; i < line_indices.size(); i++) {
        std::vector<int> column_indices;
        context.get_sonar_cart_column(line_indices[i], column_indices, neighbor_size);
        columns_indices.push_back(column_indices);
    }

    return columns_indices;
}

void SonarView_draw_columns_indices(SonarViewContext& context, std::vector<std::vector<int> > columns_indices) {
    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();

    for (size_t i = 0; i < columns_indices.size(); i++) {
        for (size_t j = 0; j < columns_indices[i].size(); j++) {
            int index = columns_indices[i][j];
            cv::circle(context.sel_canvas, context.scale_to_selection(cart_points[index]), 5, cv::Scalar(255, 0, 0), CV_FILLED);
        }
    }
}

cv::Rect_<float> SonarView_get_bounding_rect(SonarViewContext& context, const std::vector<int>& indices) {
    std::vector<cv::Point2f> sector_cart_points;
    for (size_t i = 0; i < indices.size(); i++){
        std::vector<cv::Point2f> points = context.sonar_holder.GetSectorPoints(indices[i]);
        sector_cart_points.insert(sector_cart_points.end(), points.begin(), points.end());
    }
    return sonar_processing::image_util::bounding_rect(cv::Mat(sector_cart_points));
}

int SonarView_get_minimum_bin(SonarViewContext& context, const std::vector<int>& indices) {
    int min_bin = INT_MAX;

    for (size_t i = 0; i < indices.size(); i++) {
        int bin = context.sonar_holder.index_to_bin(indices[i]);
        if (bin < min_bin) min_bin = bin;
    }

    return min_bin;
}

float SonarView_get_resolution(SonarViewContext& context, const std::vector<int>& indices, float decimal_shift = 2.0) {
    int min_bin = SonarView_get_minimum_bin(context, indices);

    float max_radius = 2 * context.sonar_holder.bin_count() * M_PI * context.sonar_holder.beam_width();
    float min_radius = 2 * min_bin * M_PI * context.sonar_holder.beam_width();

    float shift = powf(10.0, decimal_shift);
    float res = min_radius / (max_radius * shift);

    return res;
}

void SonarView_get_sector_polar_limits(SonarViewContext& context, int index, float& start_radius, float& final_radius, float& start_theta, float& final_theta) {
    int bin = context.sonar_holder.index_to_bin(index);
    int beam = context.sonar_holder.index_to_beam(index);
    start_radius = bin+0;
    final_radius = bin+1;
    start_theta = context.sonar_holder.beam_value_at(beam+0);
    final_theta = context.sonar_holder.beam_value_at(beam+1);
}

void SonarView_vertical_cart_sector_limit(SonarViewContext& context, int index, float resolution, const cv::Point2f& ref_point, cv::Point2f& start_point, cv::Point2f& final_point, SCANNING_DIRECTION direction = SCANNING_HORIZONTAL) {
    float r0, r1;
    float t0, t1;

    SonarView_get_sector_polar_limits(context, index, r0, r1, t0, t1);

    std::vector<cv::Point2f> points = context.sonar_holder.GetSectorPoints(index);
    cv::Rect_<float> rc = sonar_processing::image_util::bounding_rect(cv::Mat(points));

    cv::Point2f origin = context.sonar_holder.cart_origin();

    start_point = cv::Point2f(-1, -1);
    final_point = cv::Point2f(-1, -1);

    bool within_sector = false;

    float start_position, final_position, scanning_position;

    if (direction == SCANNING_HORIZONTAL) {
        start_position = rc.tl().x;
        final_position = rc.br().x+resolution;
        scanning_position = ref_point.y;
    }
    else if (direction == SCANNING_VERTICAL) {
        start_position = rc.tl().y;
        final_position = rc.br().y+resolution;
        scanning_position = ref_point.x;
    }

    for (float position = start_position; position <= final_position; position+=resolution) {
        cv::Point2f pt;

        if (direction == SCANNING_HORIZONTAL) {
            pt = cv::Point2f(position, scanning_position);
        }
        else {
            pt = cv::Point2f(scanning_position, position);
        }

        float dx = origin.x - pt.x;
        float dy = origin.y - pt.y;
        float r = sqrt(dx * dx + dy * dy);
        float t = atan2(dy, dx) - M_PI_2;

        if (r >= r0 && r <= r1 && t >= t0 && t <= t1) {
            if (start_point.x == -1 && start_point.y == -1) start_point = pt;
            within_sector = true;
        }
        else {
            if (within_sector) final_point = pt;
            within_sector = false;
        }
    }

}

std::vector<cv::Point2f> SonarView_detect_intercept_line(SonarViewContext& context, const std::vector<int>& indices, const cv::Point2f& ref_cart_point, SCANNING_DIRECTION direction) {

    float resolution = SonarView_get_resolution(context, indices);

    std::vector<float> bearings = context.sonar_holder.bearings();
    cv::Point2f origin = context.sonar_holder.cart_origin();

    std::vector<cv::Point2f> start_points;
    std::vector<cv::Point2f> final_points;

    for (size_t i = 0; i < indices.size(); i++) {

        cv::Point2f start_point;
        cv::Point2f final_point;

        SonarView_vertical_cart_sector_limit(context, indices[i], resolution, ref_cart_point, start_point, final_point, direction);

        start_points.push_back(start_point);
        final_points.push_back(final_point);
    }

    std::vector<cv::Point2f> result_points;

    if (direction == SCANNING_HORIZONTAL) {
        for (size_t i = 0; i < indices.size(); i++) {
            result_points.push_back(cv::Point2f(start_points[i].x + (final_points[i].x - start_points[i].x) / 2, ref_cart_point.y));
        }
    }
    else {
        for (size_t i = 0; i < indices.size(); i++) {
            result_points.push_back(cv::Point2f(ref_cart_point.x, start_points[i].y + (final_points[i].y - start_points[i].y) / 2));
        }
    }

    return result_points;
}

void SonarView_select_image_draw_points(SonarViewContext& context, const std::vector<int>& indices, int radius, cv::Scalar color, int thickness) {
    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();
    for (size_t i = 0; i < indices.size(); i++) {
        cv::circle(context.sel_canvas, context.scale_to_selection(cart_points[indices[i]]), radius, color, thickness);
    }
}

void SonarView_select_image_draw_points(SonarViewContext& context, const std::vector<cv::Point2f>& points, int radius, cv::Scalar color, int thickness) {
    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();
    for (size_t i = 0; i < points.size(); i++) {
        cv::circle(context.sel_canvas, points[i], radius, color, thickness);
    }
}

void SonarView_selection_image_right_mouse_button_up(SonarViewContext& context, int x, int y) {
    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;

    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();

    std::vector<int> indices;
    context.find_min_distance_points(cv::Point2f(click_point.x, click_point.y), cart_points, indices, 2);

    std::vector<cv::Point2f> sector_cart_points;

    for (size_t i = 0; i < indices.size(); i++) {

        int bin = context.sonar_holder.index_to_bin(indices[i]);
        int beam = context.sonar_holder.index_to_beam(indices[i]);

        float r0 = bin+0;
        float r1 = bin+1;
        float t0 = context.sonar_holder.bearings()[beam+0];
        float t1 = context.sonar_holder.bearings()[beam+1];

        float dx = context.sonar_holder.cart_origin().x - click_point.x;
        float dy = context.sonar_holder.cart_origin().y - click_point.y;
        float r = sqrt(dx * dx + dy * dy);
        float t = atan2(dy, dx) - M_PI_2;

        if (r >= r0 && r <= r1 && t >= t0 && t <= t1) {
            context.sel_polar_index = indices[i];
            break;
        }
    }

    if (context.sel_polar_index != -1) {

        const int neighbor_size = 11;

        std::vector<int> line_indices, column_indices, neighborhood_indices;
        std::vector<cv::Point2f> line_points, column_points, neighborhood_points;

        // sonar_processing::basic_operations::intersetion_line(context.sonar_holder, context.sel_polar_index, neighbor_size, line_indices, line_points);
        // sonar_processing::basic_operations::intersetion_column(context.sonar_holder, context.sel_polar_index, neighbor_size, column_indices, column_points);
        // sonar_processing::basic_operations::neighborhood(context.sonar_holder, context.sel_polar_index, neighbor_size, neighborhood_indices, neighborhood_points);

        for (int line = 0; line < neighbor_size; line++) {
            for (int col = 0; col < neighbor_size; col++) {

                int index = neighborhood_indices[line * neighbor_size + col];
                cv::Point2f point = neighborhood_points[line * neighbor_size + col];

                if (index != -1) {
                    cv::circle(context.sel_canvas, context.scale_to_selection(cv::Point2f(point.x, column_points[line].y)), 3, cv::Scalar(0, 255, 0), CV_FILLED);
                }
            }
        }

        SonarView_select_image_draw_points(context, context.scale_to_selection(line_points), 3, cv::Scalar(255, 255, 255), CV_FILLED);
    }
}

void SonarView_selection_image_ctrl_left_mouse_button_up(SonarViewContext& context, int x, int y) {
    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;
    context.cart_to_polar(click_point.x, click_point.y, bin, beam);

    int polar_index = beam * context.sonar_holder.bin_count() + bin;

    cv::Point2f sel_cart_point = context.sonar_holder.cart_center_point(bin, beam);
    sel_cart_point = context.scale_to_selection(sel_cart_point);

    std::vector<int> directions_mapping;
    std::vector<int> neighbors_indices;
    std::vector<float> neighbors_angles;

    std::vector<float> bearings = context.sonar_holder.bearings();

    int angle_step = context.sonar_holder.beam_count() / 5;
    float rotate_angle = ((beam /  angle_step) - 2) * (M_PI / 4.0);

    // context.compute_neighbors_direction(polar_index, directions_mapping, neighbors_indices, neighbors_angles);

    const int neighbor_size = 3;
    const int total_neighbors = neighbor_size * neighbor_size;

    for (size_t i = 0; i < total_neighbors; i++) {
        int y = i / neighbor_size;
        int x = i % neighbor_size;

        printf("[%d]", neighbors_indices[i]);

        if (x == neighbor_size-1) {
            printf("\n");
        }
    }
    printf("\n");

    std::vector<int> rotate_neighbors(total_neighbors, -1);
    int cx = 1;
    int cy = 1;
    char buff[256];
    std::vector<cv::Point2f> cart_center_points = context.sonar_holder.cart_center_points();

    for (size_t i = 0; i < total_neighbors; i++) {
        int y = i / neighbor_size;
        int x = i % neighbor_size;

        int dx = cx - x;
        int dy = cy - y;
        float r = sqrt(dx * dx + dy * dy);
        float t = atan2(dy, dx) + M_PI;

        float new_angle = base::Angle::normalizeRad(t - rotate_angle);

        int xx = round(cos(new_angle) * r) + cx;
        int yy = round(sin(new_angle) * r) + cy;

        int ii = yy * neighbor_size + xx;
        rotate_neighbors[i] = ii;
        printf("[%d]", neighbors_indices[ii]);

        if (x == neighbor_size-1) {
            printf("\n");
        }
    }

    printf("direction_neighborhood\n");
    int directions_indices[4] = {1, 7, 3, 5};

    for (size_t i = 0; i < 4; i++)
    {
        int j = directions_indices[i];
        cv::Point2f point = cart_center_points[neighbors_indices[rotate_neighbors[j]]];

        point = context.scale_to_selection(point);
        int y = j / neighbor_size;
        int x = j % neighbor_size;

        int dx = cx - x;
        int dy = cy - y;
        float r = sqrt(dx * dx + dy * dy);
        float t = atan2(dy, dx) + M_PI;

        sprintf(buff, "%0.2f", base::Angle::rad2Deg(base::Angle::normalizeRad(t)));
        SonarView_draw_arrowed_line(context.sel_canvas, sel_cart_point, point, cv::Scalar(0, 255, 255), 2);
        cv::putText(context.sel_canvas, buff, point, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 127, 255));
    }
}

std::vector<int> SonarView_get_polar_corners(SonarViewContext& context, int polar_index) {
    int beam = context.sonar_holder.index_to_beam(polar_index);
    int bin = context.sonar_holder.index_to_bin(polar_index);
    int bin_count = context.sonar_holder.bin_count();

    std::vector<int> polar_indices(4, -1);
    polar_indices[0] = polar_index;
    polar_indices[1] = (beam+1)*bin_count+bin;
    polar_indices[2] = beam*bin_count+(bin+1);
    polar_indices[3] = (beam+1)*bin_count+(bin+1);
    return polar_indices;
}

std::vector<cv::Point2f> SonarView_get_cart_points(SonarViewContext& context, const std::vector<int>& polar_indices) {
    std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_points();
    std::vector<cv::Point2f> result_cart_points(polar_indices.size());
    for (size_t i = 0; i < polar_indices.size(); i++) {
        result_cart_points[i] = cart_points[polar_indices[i]];
    }
    return result_cart_points;
}

void SonarView_draw_polar_corners(SonarViewContext& context, int polar_index) {
    std::vector<int> polar_indices = SonarView_get_polar_corners(context, polar_index);
    std::vector<cv::Point2f> cart_points = SonarView_get_cart_points(context, polar_indices);
    for (size_t i = 0; i < cart_points.size(); i++){
        cart_points[i] = context.scale_to_selection(cart_points[i]);
        cv::circle(context.sel_canvas, cart_points[i], 2, cv::Scalar(255, 0, 0), 2);
    }
}

void SonarView_selection_image_ctrl_right_mouse_button_up(SonarViewContext& context, int x, int y) {

    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;
    context.cart_to_polar(click_point.x, click_point.y, bin, beam);

    int polar_index = beam * context.sonar_holder.bin_count() + bin;
    SonarView_draw_polar_corners(context, polar_index);

    std::vector<cv::Point2f> cart_center_points = context.sonar_holder.cart_center_points();

    if (context.sel_polar_index == -1) {
        context.sel_polar_index = polar_index;
    }

    cv::Point2f sel_cart_point = cart_center_points[polar_index];
    sel_cart_point = context.scale_to_selection(sel_cart_point);
    cv::circle(context.sel_canvas, sel_cart_point, 2, cv::Scalar(0, 255, 0), 2);

    cv::Point2f ref_cart_point = cart_center_points[context.sel_polar_index];
    ref_cart_point = context.scale_to_selection(ref_cart_point);
    cv::circle(context.sel_canvas, ref_cart_point, 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(context.sel_canvas, ref_cart_point, 2, cv::Scalar(255, 255, 255), 2);

    std::vector<int> neighbors_indices;
    std::vector<float> neighbors_angles;

    // context.sonar_holder.GetNeighborhoodAngles(context.sel_polar_index, polar_index, neighbors_indices, neighbors_angles);

    if (neighbors_indices.size() > 0) {

        char buff[256];

        std::vector<cv::Point2f> corner_indices;

        float min_dy = FLT_MAX;
        int min_index = -1;

        for (size_t i = 0; i < neighbors_indices.size(); i++) {

            cv::Point2f point = cart_center_points[neighbors_indices[i]];

            point = context.scale_to_selection(point);

            if (point.x > sel_cart_point.x) {
                float dy = ref_cart_point.y - point.y;

                printf("delta_y: %f\n", fabs(dy));
                if (fabs(dy) < min_dy) {
                    min_dy = fabs(dy);
                    min_index = i;
                }

                cv::circle(context.sel_canvas, point, 2, cv::Scalar(0, 0, 255), 2);

                std::vector<int> polar_indices = SonarView_get_polar_corners(context, neighbors_indices[i]);
                std::vector<cv::Point2f> corner_cart_points = SonarView_get_cart_points(context, polar_indices);
                corner_indices.insert(corner_indices.end(), corner_cart_points.begin(), corner_cart_points.end());

                SonarView_draw_polar_corners(context, neighbors_indices[i]);
            }
        }

        cv::Rect rc = cv::boundingRect(cv::Mat(corner_indices));
        std::cout << "rc: " << rc << std::endl;
        rc = context.scale_to_selection(rc);
        cv::rectangle(context.sel_canvas, rc, cv::Scalar(255, 0, 0), 2);
        cv::line(context.sel_canvas, ref_cart_point, cv::Point2f(rc.br().x, ref_cart_point.y), cv::Scalar(0, 255, 255), 2);

        if (min_index != -1) {
            printf("draw in_index");
            cv::Point2f point = cart_center_points[neighbors_indices[min_index]];
            point = context.scale_to_selection(point);
            cv::circle(context.sel_canvas, point, 5, cv::Scalar(0, 255, 0), 3);
        }

    }
}

void SonarView_selection_image_ctrl_shift_left_mouse_button_up(SonarViewContext& context, int x, int y) {
    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;
    context.cart_to_polar(click_point.x, click_point.y, bin, beam);

    context.sel_polar_index = beam * context.sonar_holder.bin_count() + bin;
    context.sel_cart_point = context.sonar_holder.cart_center_point(bin, beam);
    context.sel_cart_point = context.scale_to_selection(context.sel_cart_point);
    SonarView_draw_cart_path2(context);
    printf("Recursive right bins\n");
}

void SonarView_selection_image_mousecb(int event, int x, int y, int flags, void* data) {
    SonarViewContext *ctx = (SonarViewContext*)data;

    switch (event) {
        case cv::EVENT_LBUTTONUP:
        {
            ctx->sel_image.copyTo(ctx->sel_canvas);
            if (flags & cv::EVENT_FLAG_CTRLKEY) {
                if (flags & cv::EVENT_FLAG_SHIFTKEY) {
                    SonarView_selection_image_ctrl_shift_left_mouse_button_up(*ctx, x, y);
                }
                else {
                    SonarView_selection_image_ctrl_left_mouse_button_up(*ctx, x, y);
                }
            }
            else {
                SonarView_selection_image_left_mouse_button_up(*ctx, x, y);
            }
        }
        break;
        case cv::EVENT_RBUTTONUP:
        {
            ctx->sel_image.copyTo(ctx->sel_canvas);
            if (flags & cv::EVENT_FLAG_CTRLKEY) {
                SonarView_selection_image_ctrl_right_mouse_button_up(*ctx, x, y);
            }
            else {
                SonarView_selection_image_right_mouse_button_up(*ctx, x, y);

            }
        }
        break;
    }

}

void SonarView_create_selection_image(SonarViewContext& context) {
    cv::Point2f origin = context.sonar_holder.cart_origin();
    origin = context.scale_to_selection(origin);

    std::vector<cv::Point2f> pts(4);

    std::vector<float> bins = context.sonar_holder.bins();
    std::vector<float> bearings = context.sonar_holder.bearings();

    cv::Mat sel_image = context.sel_image;
    sel_image.setTo(0);

    int sel_width = sel_image.cols;
    int sel_height = sel_image.rows;
    int beam_count = context.sonar_holder.beam_count();
    int bin_count = context.sonar_holder.bin_count();

    int n = context.sonar_holder.cart_size().width / beam_count;
    cv::Rect rc = context.transform_rect;

    rc.x -= n;
    rc.y -= n;
    rc.width = rc.width + n * 2;
    rc.height = rc.height + n * 2;

    float approx_beam_size = context.sel_approx_beam_size() * context.sel_scale_factor;
    bool draw_grid = (approx_beam_size >= 5.0);

    for (size_t beam = 0; beam < beam_count-1; beam++) {
        for (size_t bin = 0; bin < bin_count-1; bin++) {

            cv::Point2f pt = context.sonar_holder.cart_point(bin, beam);

            if (pt.x >= rc.x && pt.x < rc.x + rc.width &&
                pt.y >= rc.y && pt.y < rc.y + rc.height) {

                std::vector<cv::Point2f> pts(4);
                pts[0] = context.sonar_holder.cart_point(bin, beam);
                pts[1] = context.sonar_holder.cart_point(bin+1, beam);
                pts[2] = context.sonar_holder.cart_point(bin, beam+1);
                pts[3] = context.sonar_holder.cart_point(bin+1, beam+1);

                for (size_t i = 0; i < pts.size(); i++) pts[i] = context.scale_to_selection(pts[i]);

                cv::Rect rc = cv::boundingRect(cv::Mat(pts));

                float r0 = (bin+0) * context.sel_scale_factor;
                float r1 = (bin+1) * context.sel_scale_factor;
                float t0 = bearings[beam+0];
                float t1 = bearings[beam+1];

                for (int y = rc.tl().y; y <= rc.br().y; y++) {
                    for (int x = rc.tl().x; x <= rc.br().x; x++) {

                        if (x >= 0 && x < sel_width && y >= 0 && y < sel_height) {
                            float dx = origin.x - x;
                            float dy = origin.y - y;
                            float r = sqrt(dx * dx + dy * dy);
                            float t = atan2(dy, dx) - M_PI_2;

                            if (r >= r0 && r <= r1 && t >= t0 && t <= t1) {
                                float bin_val = bins[beam * bin_count + bin];
                                if (r <= r1 && r >= (r1-3) && draw_grid) {
                                    sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 255);
                                }
                                else {
                                    sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(bin_val, bin_val, bin_val);
                                    // sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
                                }
                            }
                        }
                    }
                }

                if (draw_grid) {
                    cv::line(sel_image, cv::Point(pts[0].x, pts[0].y), cv::Point(pts[1].x, pts[1].y), cv::Scalar(0, 0, 255), 2);
                    cv::line(sel_image, cv::Point(pts[2].x, pts[2].y), cv::Point(pts[3].x, pts[3].y), cv::Scalar(0, 0, 255), 2);
                }
            }
        }
    }
}

void SonarView_process_key(SonarViewContext& context, int key) {
    float scale_factor = context.sel_rect_scale_factor;

    int trans_x = context.sel_rect_translation.x;
    int trans_y = context.sel_rect_translation.y;

    if (key == KEY_ZOOM_IN0 || key == KEY_ZOOM_IN1 || key == KEY_ZOOM_IN2) {
        scale_factor *= 1.0/1.1;
    }
    else if (key == KEY_ZOOM_OUT0 || key == KEY_ZOOM_OUT1 || key == KEY_ZOOM_OUT2) {
        scale_factor *= 1.1;
    }
    else if (key == KEY_LEFT) {
        trans_x--;
    }
    else if (key == KEY_UP) {
        trans_y--;
    }
    else if (key == KEY_RIGHT) {
        trans_x++;
    }
    else if (key == KEY_DOWN) {
        trans_y++;
    }

    if (context.is_transform_selection_rectangle(trans_x, trans_y, scale_factor)) {
        cv::Rect tr = context.transform_selection_rectangle(trans_x, trans_y, scale_factor);

        const int N = MAX_SEL_AREA / SCALE_FACTOR;
        if (tr.width < 2 || tr.width > N || tr.height < 2 || tr.height > N) {
            return;
        }

        context.transform_rect = tr;
        context.sel_rect_translation = cv::Point2f(trans_x, trans_y);
        int size = std::max<float>(tr.width, tr.height);
        context.sel_origin = cv::Point2f(tr.x, tr.y);
        context.sel_polar_index = -1;
        context.sel_rect_scale_factor = scale_factor;
        context.sel_scale_factor = (float)SEL_WINDOW_SIZE / size;
        context.draw_selection_rectangle(tr.tl() * SCALE_FACTOR, tr.br() * SCALE_FACTOR);
    }
}

void SonarView_run_selection_window(SonarViewContext& context) {
    int size = std::max<float>(context.sel_rect.width, context.sel_rect.height);
    context.sel_scale_factor = (float)SEL_WINDOW_SIZE / size;

    cv::Size sel_image_size = cv::Size(context.sel_rect.width * context.sel_scale_factor,
                                       context.sel_rect.height * context.sel_scale_factor);

    context.sel_image = cv::Mat::zeros(sel_image_size, CV_32FC3);

    SonarView_create_selection_image(context);

    context.sel_image.copyTo(context.sel_canvas);
    context.sel_rect_scale_factor = 1.0;
    context.sel_polar_index = -1;
    context.sel_rect_translation = cv::Point2f(0, 0);
    context.exit_selection_window = false;

    cv::namedWindow(SEL_WINDOW_TITLE);
    cv::setMouseCallback(SEL_WINDOW_TITLE, SonarView_selection_image_mousecb, &context);

    clock_t last_time = 0;
    bool last_key_click_state = false;

    std::vector<uchar> key_vec(KEYS, KEYS + sizeof(KEYS) / sizeof(KEYS[0]));

    while (!context.exit_selection_window) {
        // cv::rectangle(context.sel_canvas, cv::Rect(0, 0, sel_image_size.width, sel_image_size.width), cv::Scalar(255, 0, 0), 2);
        cv::imshow(SEL_WINDOW_TITLE, context.sel_canvas);
        uchar key = cv::waitKey(10);
        if (key == 27) return;
        std::vector<uchar>::iterator it = std::find(key_vec.begin(), key_vec.end(), key);
        bool is_key_click = it != key_vec.end();
        float ellapse_time = float(std::clock() - last_time) / CLOCKS_PER_SEC;

        if (ellapse_time > 0.25 && last_key_click_state) {
            std::cout << "Rebuild the selection image..." << std::endl;
            SonarView_create_selection_image(context);
            context.sel_image.copyTo(context.sel_canvas);
            last_key_click_state = false;
        }

        if (is_key_click) {
            std::cout << "Transform the selection rectangle..." << std::endl;
            SonarView_process_key(context, key);
            last_key_click_state = true;
            last_time = std::clock();
        }

    }
}

void SonarView_mousedown(SonarViewContext& context, int x, int y) {
    context.exit_selection_window = true;
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
    context.sel_point0 = cv::Point(x, y);
    std::cout << "context.sel_point0: " << context.sel_point0 << std::endl;
}

void SonarView_mousemove(SonarViewContext& context, int x, int y) {
    if (context.sel_point0.x != -1 && context.sel_point0.y != -1) {
        context.update_and_validate_selection_final_point(x, y);
        context.draw_selection_rectangle();
    }
}

void SonarView_mouseup(SonarViewContext& context, int x, int y) {
    std::cout << "SonarView_mouseup" << std::endl;
    if (context.sel_point0.x != -1 && context.sel_point0.y != -1) {
        if (context.update_and_validate_selection_final_point(x, y)) {
            context.sel_point1 = cv::Point(context.sel_point0.x+8, context.sel_point0.y+8);
            context.update_selection_rectangle();
            context.draw_selection_rectangle();
            context.sel_origin = cv::Point2f(context.sel_rect.x, context.sel_rect.y);
            context.sel_point0 = cv::Point(-1, -1);
            context.sel_point1 = cv::Point(-1, -1);
            SonarView_run_selection_window(context);
        }
        else {
            context.sel_point0 = cv::Point(-1, -1);
            context.sel_point1 = cv::Point(-1, -1);
            cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
            cv::imshow(IMAGE_TITLE, context.canvas);
        }
    }
}

void SonarView_mousecb(int event, int x, int y, int flags, void* data) {
    SonarViewContext *ctx = (SonarViewContext*)data;

    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
        {
            SonarView_mousedown(*ctx, x, y);
        }
        break;
        case cv::EVENT_LBUTTONUP:
        {
            SonarView_mouseup(*ctx, x, y);
        }
        break;
        case cv::EVENT_MOUSEMOVE:
        {
            SonarView_mousemove(*ctx, x, y);
        }
        break;
    }
}

void SonarView_initialize(SonarViewContext& context) {
    cv::Size new_size = cv::Size(context.sonar_holder.cart_size().width * SCALE_FACTOR,
                                 context.sonar_holder.cart_size().height * SCALE_FACTOR);



    std::cout << "Window Size: " << new_size << std::endl;

    cv::resize(context.sonar_holder.cart_image(), context.scale_image, new_size);
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
}

void SonarView_run(SonarViewContext& context) {
    cv::imshow(IMAGE_TITLE, context.canvas);
    cv::setMouseCallback(IMAGE_TITLE, SonarView_mousecb, (void*)&context);

    while (true) {
        char key = cv::waitKey();
        if (key == 27 || key == -1) {
            cv::destroyAllWindows();
            break;
        }
    }
}



#endif