#include <iostream>
#include <algorithm>
#include <base/samples/Sonar.hpp>
#include <base/Angle.hpp>
#include "base/MathUtil.hpp"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_target_tracking/ImageUtils.hpp"
#include "base/test_config.h"

#define GET2D(v, x, y, step) (v)[(y) * (step) + (x)]

#define SCALE_FACTOR    0.5
#define MAX_SEL_AREA    32
#define SEL_CANVAS_SIZE 512

using namespace sonar_target_tracking;

#define MIN_SEL_AREA    8
std::vector<cv::Point2f> g_cart_points;
std::vector<cv::Point2f> g_cart_center_points;
std::vector<float> g_radius;
std::vector<float> g_angles;
std::vector<float> g_bins;
std::vector<int> g_cart_to_polar;

std::vector<int> g_polar_index_left;
std::vector<int> g_polar_index_right;
std::vector<int> g_polar_index_up;
std::vector<int> g_polar_index_down;

std::vector<std::vector<int> > g_cart_to_polar_mapping;

int g_bin_count = 0;
int g_beam_count = 0;
int g_sel_bin0 = -1;
int g_sel_bin1 = -1;
int g_sel_beam0 = -1;
int g_sel_beam1 = -1;
float g_sel_area_scale_factor = 1.0;
cv::flann::Index *g_kdtree = NULL;
cv::flann::Index *g_kdtree_center = NULL;
std::vector<base::Angle> g_bearings;
cv::Point2f g_sel_image_origin;

cv::Size g_cart_size;
cv::Point2f g_cart_origin;

cv::Point g_sel_point0 = cv::Point(-1, -1);
cv::Point g_sel_point1 = cv::Point(-1, -1);

cv::Mat g_cart_mask;
cv::Mat g_cart_image;
cv::Mat g_cart_image_scaled;
cv::Mat g_canvas;
cv::Mat g_sel_image;
cv::Mat g_sel_canvas;

bool g_quit_sel_image = false;

size_t find_min_dist(std::vector<float> v, float alpha) {
    size_t index = 0;
    float min_dist = FLT_MAX;

    for (size_t i = 0; i < v.size(); i++) {
        float dist = abs(alpha - v[i]);
        if (dist > 180.0) dist = fabs(dist - 360.0);

        if (dist < min_dist) {
            index = i;
            min_dist = dist;
        }
    }

    return index;
}

void draw_sel_rectangle(cv::Point pt0, cv::Point pt1) {
    cv::cvtColor(g_cart_image_scaled, g_canvas, CV_GRAY2BGR);
    cv::rectangle(g_canvas, pt0, pt1, cv::Scalar(0, 0, 255), 2, CV_AA);
    cv::imshow("cartesian_image", g_canvas);
}

void find_cart_neighbors(int x, int y, std::vector<int>& indices, int count, cv::flann::Index *kdtree = g_kdtree) {
    std::vector<float> query;
    query.push_back((float)x);
    query.push_back((float)y);

    std::vector<float> dists;

    cv::flann::SearchParams params;
    kdtree->knnSearch(query, indices, dists, 1, params);
}

void cart_to_polar_from_neighborhood(int x, int y, int& bin, int &beam, cv::flann::Index *kdtree = g_kdtree) {
    std::vector<int> indices;
    find_cart_neighbors(x, y, indices, 1, kdtree);
    beam = indices[0] / g_bin_count;
    bin = indices[0] % g_bin_count;
}

void polar_sel_area(int x, int y, int w, int h, int& bin0, int& bin1, int& beam0, int& beam1) {
    std::vector<int> bins(4, 0);
    std::vector<int> beams(4, 0);

    cart_to_polar_from_neighborhood(x, y, bins[0], beams[0]);
    cart_to_polar_from_neighborhood(x, y+h, bins[1], beams[1]);
    cart_to_polar_from_neighborhood(x+w, y, bins[2], beams[2]);
    cart_to_polar_from_neighborhood(x+w, y+h, bins[3], beams[3]);

    bin0 = *std::min_element(bins.begin(), bins.end());
    bin1 = *std::max_element(bins.begin(), bins.end());
    beam0 = *std::min_element(beams.begin(), beams.end());
    beam1 = *std::max_element(beams.begin(), beams.end());
}

void draw_grid_image(int x, int y, int w, int h) {

    cv::Mat sel_image = cv::Mat::zeros(cv::Size(w * g_sel_area_scale_factor, h * g_sel_area_scale_factor), CV_32FC3);
    cv::Point2f origin = g_cart_origin;
    origin -= g_sel_image_origin;
    origin *= g_sel_area_scale_factor;

    std::vector<cv::Point2f> pts(4);
    size_t start_beam = (g_sel_beam0 - 1 >= 0) ? g_sel_beam0 - 1 : 0;
    for (size_t beam = start_beam; beam <= g_sel_beam1 && beam < g_beam_count - 1; beam++) {
        for (size_t bin = g_sel_bin0; bin <= g_sel_bin1 && bin < g_bin_count - 1; bin++) {

            pts[0] = GET2D(g_cart_points, bin, beam, g_bin_count);
            pts[1] = GET2D(g_cart_points, bin+1, beam, g_bin_count);
            pts[2] = GET2D(g_cart_points, bin, beam+1, g_bin_count);
            pts[3] = GET2D(g_cart_points, bin+1, beam+1, g_bin_count);

            for (size_t i = 0; i < pts.size(); i++) {
                pts[i] -= g_sel_image_origin;
                pts[i] *= g_sel_area_scale_factor;
            }
            
            cv::Rect rc = cv::boundingRect(cv::Mat(pts));

            float r0 = bin * g_sel_area_scale_factor;
            float r1 = (bin + 1) * g_sel_area_scale_factor;         
            float t0 = g_bearings[beam].getRad();
            float t1 = g_bearings[beam+1].getRad();

            for (int y = rc.tl().y; y <= rc.br().y; y++) {
                for (int x = rc.tl().x; x <= rc.br().x; x++) {
                    float dx = origin.x - x;
                    float dy = origin.y - y;
                    float r = sqrt(dx * dx + dy * dy);
                    float t = atan2(dy, dx) - M_PI_2;

                    if (r <= r1 && r >= r0 && t >= t0 && t <= t1) {
                        if (x >= 0 && x < sel_image.cols && y >= 0 && y < sel_image.rows) {
                            float bin_val = g_bins[beam * g_bin_count + bin];                            
                            if (r <= r1 && r >= (r1-1)) {
                                sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 255);
                            }
                            else {
                                sel_image.at<cv::Vec3f>(y, x) = cv::Vec3f(bin_val, bin_val, bin_val);
                            }
                        }
                    }
                }
            }
            
            cv::line(sel_image, cv::Point(pts[0].x, pts[0].y), cv::Point(pts[1].x, pts[1].y), cv::Scalar(0, 0, 255));
            cv::line(sel_image, cv::Point(pts[2].x, pts[2].y), cv::Point(pts[3].x, pts[3].y), cv::Scalar(0, 0, 255));
        }
    }

    sel_image.copyTo(g_sel_image);
}

void sel_image_mousecb(int event, int x, int y, int flags, void* data) {
    switch (event){
        case cv::EVENT_LBUTTONUP:
        {
            g_sel_image.copyTo(g_sel_canvas);

            cv::Point2f click_pt = cv::Point2f(x, y);
            click_pt += g_sel_image_origin * g_sel_area_scale_factor;
            click_pt = cv::Point2f(click_pt.x / g_sel_area_scale_factor, click_pt.y / g_sel_area_scale_factor);

            int bin, beam;
            cart_to_polar_from_neighborhood(round(click_pt.x), round(click_pt.y), bin, beam, g_kdtree_center);
            cv::Point2f center_pt = GET2D(g_cart_center_points, bin, beam, g_bin_count);

            cv::Point2f sel_center_pt = center_pt;
            sel_center_pt -= g_sel_image_origin;
            sel_center_pt *= g_sel_area_scale_factor;

            std::vector<int> neighbors(4);
            neighbors[0] = g_polar_index_left[beam * g_bin_count + bin];
            neighbors[1] = g_polar_index_right[beam * g_bin_count + bin];
            neighbors[2] = g_polar_index_up[beam * g_bin_count + bin];
            neighbors[3] = g_polar_index_down[beam * g_bin_count + bin];
            
            std::string labels[4] = {"-x", "+x", "-y", "+y"};
            
            printf("current bin: %d beam: %d\n", bin, beam);

            for (int i = 0; i < neighbors.size(); i++) {
                if (neighbors[i] != -1) {
                    int beam = neighbors[i] / g_bin_count;
                    int bin = neighbors[i] % g_bin_count;
                    cv::Point2f pt = GET2D(g_cart_center_points, bin, beam, g_bin_count);
                    cv::Point2f sel_pt = pt;
                    sel_pt -= g_sel_image_origin;
                    sel_pt *= g_sel_area_scale_factor;
                    cv::line(g_sel_canvas, sel_center_pt, sel_pt, cv::Scalar(0, 255, 0), 2);
                    cv::circle(g_sel_canvas, cv::Point(sel_pt.x, sel_pt.y), 3, cv::Scalar(0, 0, 255), 2);
                    cv::putText(g_sel_canvas, labels[i], cv::Point(sel_pt.x , sel_pt.y), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 255));
                    printf("[%d] bin: %d beam: %d\n", i, bin, beam);
                }
            }
            

        }
        break;
    }
}


void show_sel_area(cv::Point sel_point0, cv::Point sel_point1) {
    int x = std::min<int>(sel_point0.x, sel_point1.x) / SCALE_FACTOR;
    int y = std::min<int>(sel_point0.y, sel_point1.y) / SCALE_FACTOR;
    int w = abs(sel_point0.x - sel_point1.x) / SCALE_FACTOR;
    int h = abs(sel_point0.y -sel_point1.y) / SCALE_FACTOR;

    g_sel_area_scale_factor = (float)SEL_CANVAS_SIZE / std::max<float>(w, h);
    g_sel_image_origin = cv::Point2f(x, y);
    float sel_image_zoom_factor = 1.0;

    polar_sel_area(x, y, w, h, g_sel_bin0, g_sel_bin1, g_sel_beam0, g_sel_beam1);
    draw_grid_image(x, y, w, h);

    g_sel_image.copyTo(g_sel_canvas);

    cv::namedWindow("select_image");
    cv::setMouseCallback("select_image", sel_image_mousecb);

    g_quit_sel_image = false;

    while (!g_quit_sel_image) {
        cv::imshow("select_image", g_sel_canvas);

        char key = cv::waitKey(500);

        uchar ch = (uchar)key;

        float zoom_factor = sel_image_zoom_factor;
        
        int xx = x;
        int yy = y;

        if (ch == '+' || ch == 171) {
            zoom_factor *= 1.0/1.1;
        }
        else if (key == '-' || ch == 173) {
            zoom_factor *= 1.1;
        }
        else if (key == 81) {
            xx -= 1;
        }
        else if (key == 82) {
            yy -= 1;
        }
        else if (key == 83) {
            xx += 1;
        }
        else if (key == 84) {
            yy += 1;
        }

        if ( (zoom_factor != sel_image_zoom_factor) || 
             (xx != x && xx >= 0 && xx < g_cart_size.width) ||
             (yy != y && yy >= 0 && yy < g_cart_size.height) ) {

            float cx = xx + w / 2;
            float cy = yy + h / 2;

            float left = xx;
            float top = yy;
            float right = xx + w;
            float bottom = yy + h;

            left -= cx;
            top -= cy;
            right -= cx;
            bottom -= cy;

            left *= zoom_factor;
            top *= zoom_factor;
            right *= zoom_factor;
            bottom *= zoom_factor;
            
            left += cx;
            top += cy;
            right += cx;
            bottom += cy;

            float ws = right - left;
            float hs = bottom -top;
            float xs = left;
            float ys = top;

            if (ws < 2 || hs <  2 || ws > 128 || hs > 128) continue;
            
            x = xx;
            y = yy;
            g_sel_image_origin = cv::Point2f(xs, ys);
            sel_image_zoom_factor = zoom_factor;
            g_sel_area_scale_factor = (float)SEL_CANVAS_SIZE / std::max<float>(ws, hs);
            polar_sel_area(xs, ys, ws, hs, g_sel_bin0, g_sel_bin1, g_sel_beam0, g_sel_beam1);
            draw_sel_rectangle(cv::Point(xs * SCALE_FACTOR, ys * SCALE_FACTOR), cv::Point((xs + ws) * SCALE_FACTOR, (ys + hs) * SCALE_FACTOR));
            draw_grid_image(xs, ys, ws, hs);
            g_sel_image.copyTo(g_sel_canvas);

        }
    }
}
bool set_valid_point(cv::Point& pt, int x, int y) {

    int w = g_cart_size.width * SCALE_FACTOR;
    int h = g_cart_size.height * SCALE_FACTOR;

    int dx = x - g_sel_point0.x;
    int dy = y - g_sel_point0.y;
    
    if (x < 0) {
        x = 0;
    }
    else if (x > w - 1) {
        x = w - 1;
    }
    else if (abs(dx) < MAX_SEL_AREA) {
        pt.x = x;
    }
    else {
        pt.x = g_sel_point0.x + MAX_SEL_AREA * ((dx > 0) ? 1 : -1);
    }

    if (y < 0) {
        y = 0;
    }
    else if (y > h - 1) {
        y = h - 1;
    }
    else if (abs(dy) < MAX_SEL_AREA) {
        pt.y = y;
    }
    else {
        pt.y = g_sel_point0.y + MAX_SEL_AREA * ((dy > 0) ? 1 : -1);
    }

    return (abs(dx) >= MIN_SEL_AREA  && abs(dy) >= MIN_SEL_AREA);
}

void cart_image_mousecb(int event, int x, int y, int flags, void* data) {

    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
        {
            cv::cvtColor(g_cart_image_scaled, g_canvas, CV_GRAY2BGR);
            cv::destroyWindow("select_image");
            g_quit_sel_image = true;
            g_sel_point0 = cv::Point(x, y);
        }
        break;
        case cv::EVENT_LBUTTONUP:
        {
            if (g_sel_point0.x != -1 && g_sel_point0.y != -1) {
                if (set_valid_point(g_sel_point1, x, y)) {
                    cv::Point sel_point0 = g_sel_point0;
                    cv::Point sel_point1 = g_sel_point1;
                    g_sel_point0 = cv::Point(-1, -1);
                    g_sel_point1 = cv::Point(-1, -1);
                    draw_sel_rectangle(sel_point0, sel_point1);
                    show_sel_area(sel_point0, sel_point1);
                }
                else {
                    g_sel_point0 = cv::Point(-1, -1);
                    g_sel_point1 = cv::Point(-1, -1);
                    cv::cvtColor(g_cart_image_scaled, g_canvas, CV_GRAY2BGR);
                    cv::imshow("cartesian_image", g_canvas);
                }
            }
        }
        break;
        case cv::EVENT_MOUSEMOVE:
        {
            if (g_sel_point0.x != -1 && g_sel_point0.y != -1) {
                set_valid_point(g_sel_point1, x, y);
                draw_sel_rectangle(g_sel_point0, g_sel_point1);
            }
        }
        break;
    }
}


void linear_polar_to_cartesian(base::samples::Sonar sample, cv::OutputArray _dst) {
    _dst.create(g_cart_size, CV_32FC1);    
    cv::Mat dst = _dst.getMat();
    dst.setTo(0);
    float *dst_ptr = reinterpret_cast<float*>(dst.data);
    for (size_t cart_idx = 0; cart_idx < g_cart_to_polar.size(); cart_idx++) {
        if (g_cart_to_polar[cart_idx] != -1) *(dst_ptr + cart_idx) = sample.bins[g_cart_to_polar[cart_idx]];
    }
}

void weighted_polar_to_cartesian(base::samples::Sonar sample, cv::OutputArray _dst) {
    _dst.create(g_cart_size, CV_32FC1);
    cv::Mat dst = _dst.getMat();
    dst.setTo(0);

    float *dst_ptr = reinterpret_cast<float*>(dst.data);

    for (size_t cart_idx = 0; cart_idx < g_cart_to_polar.size(); cart_idx++) {

        if (g_cart_to_polar[cart_idx] != -1) {

            int polar_idx = g_cart_to_polar[cart_idx];
            int beam = polar_idx / sample.bin_count;
            int bin = polar_idx % sample.bin_count;

            if (beam < sample.beam_count-1 && bin < sample.bin_count-1) {

                float s0 = sample.bins[beam*sample.bin_count+bin+0];
                float s1 = sample.bins[beam*sample.bin_count+bin+1];
                float s2 = sample.bins[(beam+1)*sample.bin_count+bin+0];
                float s3 = sample.bins[(beam+1)*sample.bin_count+bin+1];

                float r0 = bin;
                float r1 = bin + 1;
                float t0 = sample.bearings[beam+0].getRad();
                float t1 = sample.bearings[beam+1].getRad();

                float r = g_radius[cart_idx];
                float t = g_angles[cart_idx];

                float v0 = s0 + (s1 - s0) * (r - r0);
                float v1 = s2 + (s3 - s2) * (r - r0);
                float v = v0 + (v1 - v0) * (t - t0) / (t1 - t0);

                *(dst_ptr + cart_idx) = v;
            }
        }
    }
}

void find_neighborhood(int bin, int beam) {
    std::vector<cv::Point2f> neighbors;
    std::vector<float> angles;
    std::vector<int> polar_indices;
    cv::Point2f center_pt = GET2D(g_cart_points, bin, beam, g_bin_count);

    const int neighbor_size = 3;
    for (int y = 0; y < neighbor_size; y++){
        for (int x = 0; x < neighbor_size; x++) {
            int yy = y - neighbor_size / 2;
            int xx = x - neighbor_size / 2;
            int bi = (beam+yy < 0 || beam+yy > g_beam_count) ? -1 : beam+yy;
            int bj = (bin+xx < 0 || bin+xx > g_bin_count) ? -1 : bin+xx;
            
            if ((bi != -1 && bj != -1) && (xx != 0 || yy != 0)) {
                cv::Point2f pt = GET2D(g_cart_points, bj, bi, g_bin_count);
                if (pt.x == -1 || pt.y == -1) continue;
                
                float dx = center_pt.x - pt.x;
                float dy = center_pt.y - pt.y;
                float angle = base::Angle::rad2Deg(atan2(dy, dx));

                neighbors.push_back(pt);
                angles.push_back(angle);
                polar_indices.push_back(bi * g_bin_count + bj);
            }
        }
    }

    int left_index  = (beam-1 > 0) ? polar_indices[find_min_dist(angles, 0)] : -1;
    int right_index = (beam+1 < g_beam_count-1) ? polar_indices[find_min_dist(angles, 180)] : -1;
    int up_index    = (bin-1 > 0) ? polar_indices[find_min_dist(angles, 90)] : -1;
    int down_index  = (bin+1 < g_bin_count-1) ? polar_indices[find_min_dist(angles, 270)]  : -1;

    g_polar_index_left[beam * g_bin_count + bin]  = left_index;
    g_polar_index_right[beam * g_bin_count + bin] = right_index;
    g_polar_index_up[beam * g_bin_count + bin]    = up_index;
    g_polar_index_down[beam * g_bin_count + bin]  = down_index;
}

void init_cartesian_data(base::samples::Sonar sample) {
    g_bin_count = sample.bin_count;
    g_beam_count = sample.beam_count;
    g_bearings = sample.bearings;
    g_bins = sample.bins;
    g_cart_size = cv::Size(cos(sample.beam_width.rad - M_PI_2) * sample.bin_count * 2.0, sample.bin_count);
    g_cart_origin = cv::Point2f(g_cart_size.width / 2, g_cart_size.height - 1);
    g_cart_points.assign(sample.bin_count * sample.beam_count,cv::Point2f(-1, -1));
    g_cart_center_points.assign(sample.bin_count * sample.beam_count, cv::Point2f(-1, -1));
    g_cart_to_polar.assign(g_cart_size.width * g_cart_size.height, -1);
    g_radius.assign(g_cart_size.width * g_cart_size.height, 0);
    g_angles.assign(g_cart_size.width * g_cart_size.height, 0);
    g_cart_to_polar_mapping.clear();
    
    g_polar_index_left.assign(sample.bin_count * sample.beam_count, -1);
    g_polar_index_right.assign(sample.bin_count * sample.beam_count, -1);;
    g_polar_index_up.assign(sample.bin_count * sample.beam_count, -1);;
    g_polar_index_down.assign(sample.bin_count * sample.beam_count, -1);;
    
    g_cart_mask = cv::Mat::zeros(g_cart_size, CV_8UC1);

    for (uint32_t bin = 0; bin < sample.bin_count; bin++) {
        for (uint32_t beam = 0; beam < sample.beam_count; beam++) {
            float radius = (float)bin;
            float theta = sample.bearings[beam].getRad();
            g_cart_points[beam * sample.bin_count + bin] = base::MathUtil::to_cartesianf(theta-M_PI_2, radius) + g_cart_origin;
        }
    }

    float min_angle = g_bearings[0].getRad();
    float max_angle = g_bearings[g_bearings.size()-1].getRad();
    float max_radius = sample.bin_count - 1;

    std::vector<float> bearing_radians = rock_util::Utilities::get_radians(g_bearings);
    uchar *mask_ptr = reinterpret_cast<uchar*>(g_cart_mask.data);

    std::vector<cv::Point2f> pts(4);
    for (size_t polar_idx = 0; polar_idx < g_cart_points.size(); polar_idx++) {
        int beam = polar_idx / sample.bin_count;
        int bin = polar_idx % sample.bin_count;

        find_neighborhood(bin, beam);
    
        if (beam < sample.beam_count-1 && bin < sample.bin_count-1) {
    
            pts[0] = GET2D(g_cart_points, bin, beam, sample.bin_count);
            pts[1] = GET2D(g_cart_points, bin, beam+1, sample.bin_count);
            pts[2] = GET2D(g_cart_points, bin+1, beam, sample.bin_count);
            pts[3] = GET2D(g_cart_points, bin+1, beam+1, sample.bin_count);
            cv::Rect rc = cv::boundingRect(cv::Mat(pts));
    
            float r0 = bin;
            float r1 = bin+1;
            float t0 = sample.bearings[beam].getRad();
            float t1 = sample.bearings[beam+1].getRad();

            g_cart_center_points[polar_idx] = base::MathUtil::to_cartesianf(t0 + (t1 - t0) / 2, r0 + (r1 - r0) / 2, -M_PI_2) + g_cart_origin;
    
            for (int y = rc.tl().y; y <= rc.br().y && y < g_cart_size.height; y++) {
                for (int x = rc.tl().x; x <= rc.br().x && x < g_cart_size.width; x++) {
                    size_t cart_idx = y * g_cart_size.width + x;
    
                    if (g_cart_to_polar[cart_idx] == -1) {
                        float dx = g_cart_origin.x - x;
                        float dy = g_cart_origin.y - y;
                        float r = sqrt(dx * dx + dy * dy);
                        float t = atan2(dy, dx) - M_PI_2;
    
                        g_radius[cart_idx] = r;
                        g_angles[cart_idx] = t;
    
                        if (r <= r1 && r >= r0 && t >= t0 && t <= t1) {
                            g_cart_to_polar[cart_idx] = polar_idx;
                            *(mask_ptr + cart_idx) = 255;
                        }
                    }
                }
            }
        }
    }

    cv::flann::KDTreeIndexParams indexParams;
    g_kdtree = new cv::flann::Index(cv::Mat(g_cart_points).reshape(1), indexParams);
    g_kdtree_center  = new cv::flann::Index(cv::Mat(g_cart_center_points).reshape(1), indexParams);
}

int main(int argc, char const *argv[]) {
    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/gemini-jequitaia.0.log",
        DATA_PATH_STRING +  "/logs/gemini-jequitaia.4.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.0.log",
        DATA_PATH_STRING + "/logs/gemini-ferry.3.log"
    };

    uint32_t sz = sizeof(logfiles) / sizeof(std::string);

    for (uint32_t i = 0; i < sz; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");

        base::samples::Sonar sample;
        stream.next<base::samples::Sonar>(sample);

        size_t last_bin_count = sample.bin_count;
        init_cartesian_data(sample);

         do {
            weighted_polar_to_cartesian(sample, g_cart_image);
            linear_polar_to_cartesian(sample, g_cart_image);
            cv::resize(g_cart_image, g_cart_image_scaled, cv::Size(g_cart_image.cols * SCALE_FACTOR, g_cart_image.rows * SCALE_FACTOR));
            cv::cvtColor(g_cart_image_scaled, g_canvas, CV_GRAY2BGR);
            cv::imshow("cartesian_image", g_canvas);
            cv::setMouseCallback("cartesian_image", cart_image_mousecb);

            while (true) {
                char key = cv::waitKey();
                if (key == 27 || key == -1) {
                    cv::destroyAllWindows();
                    break;
                }
            }

            stream.next<base::samples::Sonar>(sample);

            if (last_bin_count != sample.bin_count) {
                last_bin_count = sample.bin_count;
                init_cartesian_data(sample);
            }

        } while(stream.current_sample_index() < stream.total_samples());
    }

    return 0;
}
