#include <iostream>
#include <base/samples/Sonar.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_target_tracking/ImageUtils.hpp"
#include "sonar_target_tracking/SonarHolder.hpp"
#include "base/test_config.h"

#define SCALE_FACTOR     0.5
#define MIN_SEL_AREA     8
#define MAX_SEL_AREA     128
#define IMAGE_TITLE      "cartesian_image"
#define SEL_WINDOW_SIZE  512
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

using namespace sonar_target_tracking;

struct SonarViewContext {
    SonarViewContext(sonar_target_tracking::SonarHolder _sonar_holder)
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

    bool exit_selection_window;

    cv::flann::Index cart_points_kdtree;

    sonar_target_tracking::SonarHolder sonar_holder;

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

    cv::Rect_<float> scale_to_selection(cv::Rect rc) {
        return cv::Rect_<float>(scale_to_selection(rc.tl()), scale_to_selection(rc.br()));
    }

    float sel_approx_beam_size() {
        int dy = sonar_holder.cart_origin().y - transform_rect.br().y;
        int dx = sonar_holder.cart_origin().x - transform_rect.br().x;
        int radius = sqrt(dx * dx + dy * dy);
        return (cos(sonar_holder.beam_width() - M_PI_2) * radius * 2) / sonar_holder.beam_count() ;
    }

    void cart_to_polar(int x, int y, int& bin_out, int& beam_out) {

        bin_out = 0.0;
        beam_out = 0.0;

        cv::Point2f point(x, y);
        std::vector<cv::Point2f> cart_points = sonar_holder.cart_center_points();

        float min_dist = FLT_MAX;
        for (int bin = 0; bin < sonar_holder.bin_count(); bin++) {
            for (int beam = 0; beam < sonar_holder.beam_count(); beam++) {
                int idx = beam * sonar_holder.bin_count() + bin;
                float dx = point.x - cart_points[idx].x;
                float dy = point.y - cart_points[idx].y;
                float dist = sqrt(dx * dx + dy * dy);

                if (dist < min_dist) {
                    min_dist = dist;
                    bin_out = bin;
                    beam_out = beam;
                }
            }
        }
    }

    void compute_neighbors_direction(int polar_index, std::vector<int>& direction_mapping, std::vector<int>& neighbors_indices, std::vector<float>& neighbors_angles) {
        const int kNumberOfNeighbors = 4;

        sonar_holder.GetNeighborhoodAngles(polar_index, polar_index, neighbors_indices, neighbors_angles);
        direction_mapping.assign(kNumberOfNeighbors, -1);

        float direction_angles[4] = {
            0,              // 0deg
            M_PI,           // 180deg
            M_PI_2,         // 90deg
            M_PI + M_PI_2   // 270deg
        };

        for (size_t k = 0; k < kNumberOfNeighbors; k++) {
            direction_mapping[k] = sonar_holder.GetMinAngleDistance(neighbors_angles, neighbors_indices, direction_angles[k]);
        }
    }

    int min_angle_difference_element(const std::vector<int>& neighbors_indices,
                                     const std::vector<float>& neighbors_angles,
                                     float angle) {
        float min_theta = FLT_MAX;
        int min_idx = -1;
        for (size_t i = 0; i < neighbors_indices.size(); i++) {
            if (neighbors_indices[i] != -1) {
                float theta = base::MathUtil::angle_difference(neighbors_angles[i], angle);
                if (theta < min_theta) {
                    min_theta = theta;
                    min_idx = i;
                }
            }
        }
        return min_idx;

    }

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
        std::vector<float> neighbors_angles;

        sonar_holder.GetNeighborhoodAngles(ref_polar_index, polar_index, neighbors_indices, neighbors_angles);

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

    void compute_right_bins(int polar_index, std::vector<int>& indices, std::vector<float>& angles) {
        std::vector<int> neighbors_indices;
        std::vector<float> neighbors_angles;

        int count = 0;
        int current_index = polar_index;

        std::vector<cv::Point2f> cart_center_points = sonar_holder.cart_center_points();

        do {
            if (current_index == -1) break;

            cv::Point2f pt = cart_center_points[current_index];

            if (pt.x == -1 || pt.y == -1) break;

            indices.push_back(current_index);
            int beam = sonar_holder.index_to_beam(current_index);
            int bin = sonar_holder.index_to_bin(current_index);

            if (beam >= sonar_holder.beam_count() - 2 || bin >= sonar_holder.bin_count() - 2) break;

            sonar_holder.GetNeighborhoodAngles(polar_index, current_index, neighbors_indices, neighbors_angles);

            for (size_t i = 0; i < neighbors_indices.size(); i++) {
                if (cart_center_points[neighbors_indices[i]].x <= pt.x) {
                    neighbors_indices[i] = -1;
                }
            }

            int min_idx = min_angle_difference_element(neighbors_indices, neighbors_angles, M_PI);

            current_index = neighbors_indices[min_idx];
            angles.push_back(neighbors_angles[min_idx]);
            count++;

        } while (count < 2000);
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

void SonarView_draw_angles(SonarViewContext& context, int polar_index) {
    std::vector<int> neighbors_indices;
    std::vector<float> neighbors_angles;

    int sel_polar_index = (context.sel_polar_index != -1) ? context.sel_polar_index : polar_index;
    context.sonar_holder.GetNeighborhoodAngles(sel_polar_index, polar_index, neighbors_indices, neighbors_angles);

    int element_mask[] = {1, 1, 1,
                          1, 0, 1,
                          1, 1, 1};

    std::vector<cv::Point2f> pts;
    std::vector<uint32_t> indices;
    std::vector<cv::Point2f> cart_center_points = context.sonar_holder.cart_center_points();
    for (size_t i = 0; i < neighbors_indices.size(); i++) {

        if (element_mask[i]) {
            int polar_index = neighbors_indices[i];
            cv::Point2f pt = cart_center_points[polar_index];
            pt= context.scale_to_selection(pt);
            cv::circle(context.sel_canvas, pt, 3, cv::Scalar(0, 0, 255), 2);
            pts.push_back(pt);
            indices.push_back(i);
        }
    }

    char buff[256];
    for (size_t i = 0; i < indices.size(); i++){
        sprintf(buff, "%0.2f", base::Angle::rad2Deg(neighbors_angles[indices[i]]));
        cv::putText(context.sel_canvas, buff, pts[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255));
    }
}

void SonarView_draw_right_indices_line(SonarViewContext& context) {
    std::vector<int> indices = context.sel_right_indices;

    if (indices.size() > 1) {
        std::vector<cv::Point2f> cart_points = context.sonar_holder.cart_center_points();

        cv::Point2f pt = cart_points[context.sel_polar_index];
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

            float dx = pt1.x - pt0.x;

            cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt0.x, pt.y), cv::Scalar(255, 255, 0), 1);

            cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), cv::Point2f(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2);
            cv::line(context.sel_canvas, cv::Point2f(pt0.x, pt.y), cv::Point2f(pt1.x, pt.y), cv::Scalar(0, 255, 255), 2);

            cv::circle(context.sel_canvas, cv::Point2f(pt0.x, pt0.y), 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(context.sel_canvas, cv::Point2f(pt1.x, pt1.y), 2, cv::Scalar(0, 0, 255), 2);

            cv::circle(context.sel_canvas, cv::Point2f(pt0.x, pt.y), 1, cv::Scalar(255, 0, 0), 2);
            cv::circle(context.sel_canvas, cv::Point2f(pt1.x, pt.y), 1, cv::Scalar(255, 0, 0), 2);
        }
    }
}

void SonarView_initialize_right_indices(SonarViewContext& context) {
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);

    std::vector<float> angles;
    context.sel_right_indices.clear();
    context.compute_right_bins(context.sel_polar_index, context.sel_right_indices, angles);
}

void SonarView_draw_cart_path(SonarViewContext& context) {
    SonarView_initialize_right_indices(context);
    SonarView_draw_right_indices_line(context);

    cv::circle(context.sel_canvas, cv::Point(context.sel_cart_point.x, context.sel_cart_point.y), 3, cv::Scalar(0, 255, 0), 2);

    cv::imshow(SEL_WINDOW_TITLE, context.sel_canvas);
    cv::imshow(IMAGE_TITLE, context.canvas);
}

void SonarView_draw_cart_path2(SonarViewContext& context) {
    cv::cvtColor(context.scale_image, context.canvas, CV_GRAY2BGR);
    std::vector<float> angles;
    context.sel_right_indices.clear();
    context.cart_right_bins(context.sel_polar_index, context.sel_right_indices);
    SonarView_draw_right_indices_line(context);
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

void SonarView_selection_image_right_mouse_button_up(SonarViewContext& context, int x, int y) {
    int beam, bin;
    cv::Point2f click_point = cv::Point2f(x / context.sel_scale_factor, y /context.sel_scale_factor);
    click_point += context.sel_origin;
    context.cart_to_polar(click_point.x, click_point.y, bin, beam);

    int polar_index = beam * context.sonar_holder.bin_count() + bin;
    context.sel_polar_index = polar_index;

    cv::Point2f sel_cart_point = context.sonar_holder.cart_center_point(bin, beam);
    sel_cart_point = context.scale_to_selection(sel_cart_point);

    cv::circle(context.sel_canvas, sel_cart_point, 3, cv::Scalar(0, 255, 0), 2);

    SonarView_draw_angles(context, polar_index);
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

    context.compute_neighbors_direction(polar_index, directions_mapping, neighbors_indices, neighbors_angles);

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

    context.sonar_holder.GetNeighborhoodAngles(context.sel_polar_index, polar_index, neighbors_indices, neighbors_angles);

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
    bool draw_grid = (approx_beam_size >= 40.0);

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
                                if (r <= r1 && r >= (r1-1) && draw_grid) {
                                    sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 255);
                                }
                                else {
                                    // sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(bin_val, bin_val, bin_val);
                                    sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
                                }
                            }
                        }
                    }
                }

                if (draw_grid) {
                    cv::line(sel_image, cv::Point(pts[0].x, pts[0].y), cv::Point(pts[1].x, pts[1].y), cv::Scalar(0, 0, 255));
                    cv::line(sel_image, cv::Point(pts[2].x, pts[2].y), cv::Point(pts[3].x, pts[3].y), cv::Scalar(0, 0, 255));
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
}

void SonarView_mousemove(SonarViewContext& context, int x, int y) {
    if (context.sel_point0.x != -1 && context.sel_point0.y != -1) {
        context.update_and_validate_selection_final_point(x, y);
        context.draw_selection_rectangle();
    }
}

void SonarView_mouseup(SonarViewContext& context, int x, int y) {
    if (context.sel_point0.x != -1 && context.sel_point0.y != -1) {
        if (context.update_and_validate_selection_final_point(x, y)) {
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

int main(int argc, char const *argv[]) {
    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    sonar_target_tracking::SonarHolder sonar_holder;

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);

        do {

            sonar_holder.Reset(sample.bins,
                rock_util::Utilities::get_radians(sample.bearings),
                sample.beam_width.getRad(),
                sample.bin_count,
                sample.beam_count);
                stream.next<base::samples::Sonar>(sample);

            SonarViewContext context(sonar_holder);
            SonarView_initialize(context);
            SonarView_run(context);

        } while(stream.current_sample_index() < stream.total_samples());

        break;
    }

    return 0;
}
