//
// Created by cjyhd on 2020/11/6.
//
#include "library.h"
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
cv::Mat src;

int main(void)
{
    cv::Mat dst_log, gray_src, imageEnhance;

    src = cv::imread(R"(box_mask.png)");
    if(src.empty())
    {
        std::cout << "image is empty!!" << std::endl;
    }
    cv::namedWindow("原始图片",cv::WINDOW_AUTOSIZE);
    cv::imshow("原始图片",src);
    cv::waitKey(1000);

    cv::cvtColor(src, gray_src,cv::COLOR_BGR2GRAY);
    GaussianBlur(gray_src, gray_src, Size(3, 3), 0, 0, BORDER_DEFAULT);	//先通过高斯模糊去噪声

    // 掩膜增强
    cv::Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, 0, 5, 0, 0, -1, 0);
    cv::filter2D(gray_src, imageEnhance, CV_8UC3, kernel);
    GaussianBlur(imageEnhance, imageEnhance, Size(3, 3), 0, 0, BORDER_DEFAULT);	//先通过高斯模糊去噪声
    houghLine_display(src, imageEnhance, 125.1, 558.021, cv::Point(686, 378));
    cv::waitKey(0);
}
