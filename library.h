#ifndef HOUGH_LINE_LIB_LIBRARY_H
#define HOUGH_LINE_LIB_LIBRARY_H
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <thread>

using namespace cv;
using namespace std;

namespace HoughLine_lib{
    typedef struct lineparas_struct{
        float A;
        float B;
        float C;
    } LineParas_Struct;

    typedef struct lineparask_struct{
        LineParas_Struct paras;
        float k;
        float distance;
    } LineParasK_Struct;
/*
 * parallelisim平行度，两个方向向量越平行点乘值越大
 */
    typedef struct updownline_struct{
        LineParasK_Struct up_line;
        LineParasK_Struct down_line;
        float parallelisim;

        bool operator==(const updownline_struct b) const
        {
            return this->parallelisim == b.parallelisim;
        }
        bool operator!=(const updownline_struct b) const
        {
            return this->parallelisim != b.parallelisim;
        }
        bool operator<=(const updownline_struct b) const
        {
            return this->parallelisim <= b.parallelisim;
        }
        bool operator>=(const updownline_struct b) const
        {
            return this->parallelisim >= b.parallelisim;
        }
        bool operator<(const updownline_struct b) const
        {
            return this->parallelisim < b.parallelisim;
        }
        bool operator>(const updownline_struct b) const
        {
            return this->parallelisim > b.parallelisim;
        }
    } UpDownLine_Struct;
}

class HoughLine{
public:
    HoughLine(float _img_len, float _depth_distance, cv::Point _center_point);

public:
    static std::vector<HoughLine_lib::LineParasK_Struct> computeLineParas(const std::vector<cv::Vec4f>& _lines);
    static inline float distancePoint2Line(const HoughLine_lib::LineParasK_Struct& _line, const cv::Point& _center_point);
    static inline bool isCenterUpLine(const HoughLine_lib::LineParasK_Struct& _line, const cv::Point& _center_point);
    HoughLine_lib::UpDownLine_Struct line_NMS(const std::vector<HoughLine_lib::LineParasK_Struct>& _lines_parask, const cv::Point& _center_point) const;
    void line_Paint(cv::Mat& _dst, const HoughLine_lib::UpDownLine_Struct& _up_down_line) const;
    static inline float parallelLine2Line(const HoughLine_lib::updownline_struct& _line);
    void thread_function(const cv::Mat& _img_input, int _index);

public:
    std::vector<HoughLine_lib::UpDownLine_Struct> get_thread_UpDownLine_vector();
    cv::Mat get_imageEnhance();
    cv::Mat get_src();
    float get_depth_distance() const;
    float get_f_zed() const;
    bool set_src(cv::Mat _src);
    bool set_imageEnhance(cv::Mat _imageEnhance);
    float get_img_len();

private:
    float img_len;
    float img_width{};
    float img_height{};
    cv::Point center_point;
    cv::Mat src, imageEnhance;
    int cur_val;
    int hough_thres;
    int hough_minline;
    int hough_maxline;
    float f_zed{};
    float depth_distance{};
    std::vector<HoughLine_lib::UpDownLine_Struct> thread_UpDownLine_vector;
};

void houghLine_display(cv::Mat src, cv::Mat _imageEnhance, float _img_len_, float _depth_distance_, cv::Point _center_point_);
#endif //HOUGH_LINE_LIB_LIBRARY_H
