#include "library.h"

#include <iostream>
#include <utility>
//#define DEBUG

HoughLine::HoughLine(float _img_len, float _depth_distance, cv::Point _center_point) {
    img_len = _img_len;
    cur_val = static_cast<int> ((img_len) / 4);
    hough_thres = static_cast<int> ((img_len) / 4);
    hough_minline = static_cast<int> ((img_len) / 2);
    hough_maxline = static_cast<int> ((img_len) / 4);
    depth_distance = _depth_distance;
    center_point = std::move(_center_point);
    img_width = 1280;
    img_height = 720;
    f_zed = 709.70386169052335;
    depth_distance = 558.021;
    thread_UpDownLine_vector.resize(15);
}

/* 根据输入的直线两点，计算直线A B C参数和斜率K
 * _input: 直线两点数组
 * _output: 直线参数数组
 */
std::vector<HoughLine_lib::LineParasK_Struct> HoughLine::computeLineParas(const vector<cv::Vec4f> &_lines) {
    std::vector<HoughLine_lib::LineParasK_Struct> lines_paras;
    if(_lines.empty())
    {
        std::cout << "ERROR: computeLineParas --> lines is empty!" << std::endl;
        return lines_paras;
    }

    //A = Y2 - Y1
    //B = X1 - X2
    //C = X2*Y1 - X1*Y2
    HoughLine_lib::LineParasK_Struct _line_paras;
    for(const auto & _line : _lines)
    {
        float X1 = _line[0], X2 = _line[2],
                Y1 = _line[1], Y2 = _line[3];
        float A_line = Y2 - Y1;
        float B_line = X1 - X2;
        float C_line = X2 * Y1 - X1 * Y2;

        float K_line = 0;
        if(fabs(X1 - X2) < std::numeric_limits<float>::min())
        {
            K_line = std::numeric_limits<float>::max();
        }
        else
        {
            K_line = (Y2 - Y1) / (X2 - X1);
        }
        _line_paras.paras.A = A_line;
        _line_paras.paras.B = B_line;
        _line_paras.paras.C = C_line;
        _line_paras.k = K_line;

        // 存入数组中
        lines_paras.push_back(_line_paras);
    }
    return lines_paras;
}

/* 计算点到直线的距离
 * _line: 输入的直线参数、
 * _center_point: 输入的点坐标
 * _output: 返回点到直线的距离
 */
float HoughLine::distancePoint2Line(const HoughLine_lib::LineParasK_Struct &_line, const Point &_center_point) {
    float _distance_point2line = 0;
    auto _denominator = static_cast<float>(std::sqrt(std::pow(_line.paras.A, 2) +
                                   std::pow(_line.paras.B, 2)));
    if(fabs(_denominator - 0) < numeric_limits<float>::min())
    {
        std::cout << "ERROR: distancePoint2Line --> A and B are zero!" << std::endl;
        // 分母=0，说明A和B都为0，返回-1
        _distance_point2line = -1;
        return _distance_point2line;
    }
    _distance_point2line = fabs(_line.paras.A * _center_point.x +
                                _line.paras.B * _center_point.y + _line.paras.C) / _denominator;
    return _distance_point2line;
}

/* 检验输入的点是否在直线上方，若是则返回true
 * _line: 输入的直线参数、
 * _center_point: 输入的点坐标
 * _output: 在上方返回true
 */
bool HoughLine::isCenterUpLine(const HoughLine_lib::LineParasK_Struct &_line, const Point &_center_point) {
    return _line.paras.A * _center_point.x +
           _line.paras.B * _center_point.y +
           _line.paras.C > 0;
}

/* 输入直线参数，进行直线的”非极大值抑制“
 * lines: 输入直线参数数组
 * _output: 输出经过过滤的直线参数数组
 */
HoughLine_lib::UpDownLine_Struct
HoughLine::line_NMS(const vector<HoughLine_lib::LineParasK_Struct> &_lines_parask, const Point &_center_point) const {
    HoughLine_lib::UpDownLine_Struct _lines_NMS;
    if(_center_point.x > img_width || _center_point.y > img_height)
    {
        std::cout << "ERROR: line_NMS --> center is out of range!" << std::endl;
        return _lines_NMS;
    }
//    std::cout << "_center_point" << _center_point << std::endl;
    // 保存点到直线距离的最小值
    float _distance_up_min = std::numeric_limits<float>::max();
    float _distance_down_min = std::numeric_limits<float>::max();

    for(const auto & _line : _lines_parask)
    {
//        std::cout << "_line.paras" << "A: " << _line.paras.A << "B: " <<  _line.paras.B << "C: " << _line.paras.C << " " << std::endl;
        // 得到中心点到该条直线的距离
        float _distancePoint2Line = distancePoint2Line(_line, _center_point);
        if(_distancePoint2Line <0 )
        {
            std::cout << "ERROR: line_NMS --> distance < 0!" << std::endl;
            return _lines_NMS;
        }
        // 始终保存点到直线距离最短的直线 && 点是否在直线上方
        if(_distancePoint2Line < _distance_up_min && isCenterUpLine(_line, _center_point))
        {
            _lines_NMS.up_line = _line;
            // 刚push进入的line，设置中心点到直线的距离
            _lines_NMS.up_line.distance = _distancePoint2Line;
            _distance_up_min = _distancePoint2Line;
#ifdef DEBUG
            std::cout << "get up line" << std::endl;
#endif
        }
        else if(_distancePoint2Line < _distance_down_min && !isCenterUpLine(_line, _center_point))
        {
            _lines_NMS.down_line = _line;
            // 刚push进入的line，设置中心点到直线的距离
            _lines_NMS.down_line.distance = _distancePoint2Line;
            _distance_down_min = _distancePoint2Line;
#ifdef DEBUG
            std::cout << "get down line" << std::endl;
#endif
        }
        else continue;
    }
    return _lines_NMS;
}

/* 根据输入的直线参数画中心点上方和下方的直线
 *_up_down_line: 输入的直线参数
 */
void HoughLine::line_Paint(Mat &_dst, const HoughLine_lib::UpDownLine_Struct &_up_down_line) const {
    std::vector<int> x1, x2, y1, y2;
    // 若有上方直线
    if (static_cast<bool>(_up_down_line.up_line.distance)) {
        // 若x=0时，y>0，那么就取x=0
        if (-_up_down_line.up_line.paras.C / _up_down_line.up_line.paras.B > 0) {
            x1.push_back(0);
            y1.push_back(-_up_down_line.up_line.paras.C / _up_down_line.up_line.paras.B);
        } else {
            y1.push_back(0);
            x1.push_back(-_up_down_line.up_line.paras.C / _up_down_line.up_line.paras.A);
        }
        if ((-_up_down_line.up_line.paras.A * static_cast<float>(img_width) - _up_down_line.up_line.paras.C) /
            _up_down_line.up_line.paras.B < static_cast<float>(img_height)) {
            x2.push_back(img_width);
            y2.push_back((-_up_down_line.up_line.paras.A * static_cast<float>(img_width) - _up_down_line.up_line.paras.C) /
                         _up_down_line.up_line.paras.B);
        }else {
            y2.push_back(img_height);
            x2.push_back((-_up_down_line.up_line.paras.B * static_cast<float>(img_height) - _up_down_line.up_line.paras.C) /
                         _up_down_line.up_line.paras.A);
        }
    }
    else {
        std::cout << "no up line" << std::endl;
    }

    // 若有下方直线
    if (static_cast<bool>(_up_down_line.down_line.distance)) {
        // 若x=0时，y>0，那么就取x=0
        if (-_up_down_line.down_line.paras.C / _up_down_line.down_line.paras.B > 0) {
            x1.push_back(0);
            y1.push_back(-_up_down_line.down_line.paras.C / _up_down_line.down_line.paras.B);
        } else {
            y1.push_back(0);
            x1.push_back(-_up_down_line.down_line.paras.C / _up_down_line.down_line.paras.A);
        }
        if ((-_up_down_line.down_line.paras.A * static_cast<float>(img_width) - _up_down_line.down_line.paras.C) /
            _up_down_line.down_line.paras.B < static_cast<float>(img_height)) {
            x2.push_back(img_width);
            y2.push_back((-_up_down_line.down_line.paras.A * static_cast<float>(img_width) - _up_down_line.down_line.paras.C) /
                         _up_down_line.down_line.paras.B);
        }else {
            y2.push_back(img_height);
            x2.push_back((-_up_down_line.down_line.paras.B * static_cast<float>(img_height) - _up_down_line.down_line.paras.C) /
                         _up_down_line.down_line.paras.A);
        }
    }
    else {
        std::cout << "no down line" << std::endl;
    }

    cv::Scalar _color = cv::Scalar(39,0,235);
    for(std::size_t _i = 0; _i < x1.size(); _i ++) {
        cv::line(_dst,cv::Point(x1[_i],y1[_i]),cv::Point(x2[_i],y2[_i]),_color,2,cv::LINE_AA);
    }
}

/* 返回两条直线的平行度
 * _line: 输入的直线结构体
 * _output: 输出直线的平行度
 */
float HoughLine::parallelLine2Line(const HoughLine_lib::updownline_struct &_line) {
    float _parallelisim;
    // 如果存在上直线的话，肯定存在点到直线的距离，可用来判断直线是否存在
    if(static_cast<bool>(_line.up_line.distance) && static_cast<bool>(_line.down_line.distance))
    {
        cv::Point2f up_dir_vector(1 / std::sqrt(1+std::pow(_line.up_line.k, 2)),
                                  _line.up_line.k / std::sqrt(1+std::pow(_line.up_line.k, 2)));
        cv::Point2f down_dir_vector(1 / std::sqrt(1+std::pow(_line.down_line.k, 2)),
                                    _line.down_line.k / std::sqrt(1+std::pow(_line.down_line.k, 2)));
#ifdef DEBUG
//        std::cout << "up_dir_vector: " << up_dir_vector <<
//            "down_dir_vector: " << down_dir_vector <<
//            "_line.down_line.k: " << _line.down_line.k <<
//            "_line.up_line.k: " << _line.up_line.k << std::endl;
#endif
        _parallelisim = up_dir_vector.x * down_dir_vector.x +
                        up_dir_vector.y * down_dir_vector.y;
    }else{
#ifdef DEBUG
        std::cout << "_line.up_line.distance: " << _line.up_line.distance << std::endl
            << "_line.down_line.distance" << _line.down_line.distance << std::endl;
#endif
        _parallelisim = 0;
    }
    return fabs(_parallelisim);
}

/* 定义线程函数，得到上下直线
 *
 */
void HoughLine::thread_function(const Mat &_img_input, int _index) {
    cv::Mat _img_gaussianBlur, canny_dst;
    // 定义高斯核大小
    int _gauss_kernel = _index * 2 + 1;
    GaussianBlur(_img_input, _img_gaussianBlur, Size(_gauss_kernel, _gauss_kernel), 0, 0, BORDER_DEFAULT);

    //边缘检测 使用动态的边缘检测范围 检测尺寸为3 使用非精确的近似算法
    cv::Canny(_img_gaussianBlur,canny_dst,cur_val,cur_val*2,3,false);

    vector<cv::Vec4f> plines;   //定义直线向量容器  可以理解为数组 存储所有的直线概率点集合
    //直线检测 直线概率点 一个像素点为检测的步进检测步长 一度的检测角度 低于10个的直线点集合过滤
    cv::HoughLinesP(canny_dst,plines,1,CV_PI/180.0,hough_thres,hough_minline,hough_maxline);

    // 非极大值抑制
    std::vector<HoughLine_lib::LineParasK_Struct> _lines_parask;
    HoughLine_lib::UpDownLine_Struct _up_down_line;
    // 获取直线参数
    if(plines.empty()){return;}
//    std:cout << "plines para: " << plines[0] << " " << plines[1] << std::endl;
    _lines_parask = computeLineParas(plines);

    // 获得中心点上方和下方的直线
    _up_down_line = line_NMS(_lines_parask, center_point);
    // 得到直线的平行度
    float _parallelisim = parallelLine2Line(_up_down_line);
    _up_down_line.parallelisim = _parallelisim;
    thread_UpDownLine_vector.at(_index) = _up_down_line;
//    std::cout << "thread_UpDownLine_vector: " << thread_UpDownLine_vector.at(_index).up_line.k << "  " <<
//    thread_UpDownLine_vector.at(_index).down_line.k << "  " << thread_UpDownLine_vector.at(_index).parallelisim << std::endl;
}

std::vector<HoughLine_lib::UpDownLine_Struct> HoughLine::get_thread_UpDownLine_vector() {
    return thread_UpDownLine_vector;
}

cv::Mat HoughLine::get_imageEnhance() {
    return imageEnhance;
}

cv::Mat HoughLine::get_src() {
    return src;
}

float HoughLine::get_depth_distance() const {
    return depth_distance;
}

float HoughLine::get_f_zed() const {
    return f_zed;
}

bool HoughLine::set_src(cv::Mat _src) {
    src = _src;
    return true;
}

bool HoughLine::set_imageEnhance(cv::Mat _imageEnhance) {
    imageEnhance = _imageEnhance;
    return true;
}

float HoughLine::get_img_len() {
    return img_len;
}

void houghLine_display(cv::Mat _src, cv::Mat _imageEnhance, float _img_len_, float _depth_distance_, cv::Point _center_point_) {
    std::cout << "hough line start!!!!!" << std::endl;
    if(_img_len_ <=0 || _depth_distance_ <= 0 || _center_point_.x <= 0 || _center_point_.y <= 0)
    {
        std::cout << "ERROR --> houghLine_display: input error!!!!!!!!!!" << std::endl;
        return;
    }
    if(_src.empty())
    {
        std::cout << "ERROR --> houghLine_display: image is empty!!!" << std::endl;
        return;
    }
    HoughLine _houghline(_img_len_, _depth_distance_, std::move(_center_point_));
    _houghline.set_src(_src);
    _houghline.set_imageEnhance(_imageEnhance);
    // 保存最终经过筛选后的上下直线
    HoughLine_lib::updownline_struct final_up_down_line{};
    // 多线程执行
    vector <std::thread> _threads(15);
    using Func = void(HoughLine::*)(const Mat &, int);

    Func f1 = &HoughLine::thread_function;

    for(size_t _i = 0; _i < 15; _i++)
    {
        _threads.at(_i) = std::thread(f1, &_houghline, _imageEnhance, _i);
    }
    for (auto &thread : _threads)
        thread.join();
#ifdef DEBUG
    std::cout << "All T threads joined!\n";
    // 得到最大的平行度
    std::vector<HoughLine_lib::UpDownLine_Struct> _thread_UpDownLine_vector = _houghline.get_thread_UpDownLine_vector();
    auto maxPosition = max_element(_thread_UpDownLine_vector.begin(), _thread_UpDownLine_vector.end());
    std::cout << "max parallelisim: " << std::endl
        << "index: " << maxPosition - _thread_UpDownLine_vector.begin() << std::endl
        << "value: " << (*maxPosition).parallelisim << std::endl;
#endif
    std::vector<HoughLine_lib::UpDownLine_Struct> _thread_UpDownLine_vector = _houghline.get_thread_UpDownLine_vector();
//    cv::Mat dst_NMS = cv::Mat(_src.size(),_src.type());
    cv::Mat dst_NMS = _src.clone();
    float _max_paralellisim_up_down = 0;
    float _min_dis_up_down = std::numeric_limits<float>::max();
//    std::vector<HoughLine_lib::UpDownLine_Struct> _thread_UpDownLine_vector = _houghline.get_thread_UpDownLine_vector();
    float param_img_len = _houghline.get_img_len();
    for(std::size_t _i = 0; _i < _thread_UpDownLine_vector.size(); _i ++)
    {
        if(static_cast<bool>(_thread_UpDownLine_vector[_i].up_line.distance)){}else{ continue;}
        for(std::size_t _j = _i+1; _j < _thread_UpDownLine_vector.size(); _j ++)
        {
            if(static_cast<bool>(_thread_UpDownLine_vector[_j].down_line.distance)){}else{ continue;}
            HoughLine_lib::updownline_struct _up_down_line{};
            _up_down_line.up_line = _thread_UpDownLine_vector[_i].up_line;
            _up_down_line.down_line = _thread_UpDownLine_vector[_j].down_line;
            float _parallelisim_ = _houghline.parallelLine2Line(_up_down_line);
            float _dis_up_down = _up_down_line.up_line.distance + _up_down_line.down_line.distance;
//            std::cout << "_parallelisim_ is :   " <<  _parallelisim_ << std::endl;
            if(_parallelisim_ > _max_paralellisim_up_down && _dis_up_down < param_img_len/3 && _dis_up_down > param_img_len/10)
            {
                final_up_down_line = _up_down_line;
                _max_paralellisim_up_down = _parallelisim_;
                _min_dis_up_down = _dis_up_down;
                std::cout << "_min_dis_up_down: " << _min_dis_up_down << std::endl;
                std::cout << "_max_paralellisim_up_down: " << _max_paralellisim_up_down << std::endl;
            }
        }
    }
    // 如果最大的平行度还是为0，说明只有上线或者只有下线
    if(_max_paralellisim_up_down == 0)
    {
        std::cout << "ERROR: canny_demo --> only has one line" << std::endl;
    }
    // 将最终经过筛选后的直线画出来
    _houghline.line_Paint(dst_NMS, final_up_down_line);
    float _angel_lines = std::atan(final_up_down_line.up_line.k);
    float _depth_distance = _houghline.get_depth_distance();
    float _f_zed = _houghline.get_f_zed();
    float _thickness = (final_up_down_line.up_line.distance + final_up_down_line.down_line.distance) * (_depth_distance / _f_zed);
    std::cout << "Sucess!!" << "----> " << "angel: " << _angel_lines << std::endl;
    char img_text[50];
    sprintf(img_text, "Angel: %f  |  Thickness: %fmm", -_angel_lines / M_PI * 180, _thickness);
    std::cout << img_text << std::endl;
    cv::Mat cacul;
    cv::addWeighted(_src, 0.8, dst_NMS, 1, 0, cacul);
    cv::putText(cacul, img_text, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("NMS叠加",cacul);
}



