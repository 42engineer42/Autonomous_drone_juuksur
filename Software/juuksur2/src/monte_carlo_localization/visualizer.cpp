//Standard libraries
#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include <math.h>
// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//Class header
#include "visualizer.hpp"

Visualizer::Visualizer(double max_x, double max_y, double pixels_per_meter) : _max_x(max_x), _max_y(max_y), _pixels_per_meter(pixels_per_meter)
{
    _image = cv::Mat((int)(_pixels_per_meter * _max_x),
                     (int)(_pixels_per_meter * _max_y),
                     CV_8UC3,
                     cv::Scalar(255, 255, 255));
    //cv::imwrite("Images/test.jpg", _image);
    /*cv::imshow("image", _image);
    cv::waitKey(0);*/
}

void Visualizer::write_image(std::string tag)
{
    cv::imwrite("Images/img_" + tag + ".jpg", _image);
    cv::imshow("image", _image);
    cv::waitKey(0);
}

void Visualizer::draw_points(std::vector<std::pair<double, double>> points, int color)
{
    for (auto point : points)
    {
        //get_rgb(color);
        cv::ellipse(_image, cv::RotatedRect(cv::Point(point.first * _pixels_per_meter, point.second * _pixels_per_meter), cv::Size(5, 5), 0), get_bgr(color), 5);
    }
}

void Visualizer::draw_points(std::vector<std::pair<std::pair<double, double>, double>> points, int color)
{
    for (auto point : points)
    {
        //get_rgb(color);
        cv::ellipse(_image,
                    cv::RotatedRect(cv::Point(point.first.first * _pixels_per_meter, point.first.second * _pixels_per_meter),
                                    cv::Size(5, 5), 0),
                    get_bgr(color),
                    5);
        cv::line(_image,
                 cv::Point(point.first.first * _pixels_per_meter, point.first.second * _pixels_per_meter),
                 cv::Point(point.first.first * _pixels_per_meter + 7 * cos(point.second),
                           point.first.second * _pixels_per_meter + 7 * sin(point.second)),
                 get_secondary_bgr(color, 0),
                 2);
    }
}

void Visualizer::draw_points(std::vector<std::pair<std::pair<double, double>, double>> points, int color, double saturations[])
{
    for (int i=0; i<points.size(); i++)
    {
        std::pair<std::pair<double, double>, double> point = points[i];
        cv::ellipse(_image,
                    cv::RotatedRect(cv::Point(point.first.first * _pixels_per_meter, point.first.second * _pixels_per_meter),
                                    cv::Size(5, 5), 0),
                    get_weighted_bgr(color, saturations[i]),
                    5);
        cv::line(_image,
                 cv::Point(point.first.first * _pixels_per_meter, point.first.second * _pixels_per_meter),
                 cv::Point(point.first.first * _pixels_per_meter + 7 * cos(point.second),
                           point.first.second * _pixels_per_meter + 7 * sin(point.second)),
                 get_secondary_bgr(color, 0),
                 2);
    }
}

cv::Scalar Visualizer::get_bgr(int hue)
{
    return get_secondary_bgr(hue, 255);
}

cv::Scalar Visualizer::get_secondary_bgr(int hue, int value)
{
    cv::Mat3f bgr;
    cv::Vec3f hsv;
    hsv[0] = (float)hue;
    hsv[1] = (float)255;
    hsv[2] = (float)value;
    cvtColor(cv::Mat3f(3, 3, hsv), bgr, CV_HSV2BGR);
    return bgr.at<cv::Vec3f>(0, 0);
}

cv::Scalar Visualizer::get_weighted_bgr(int hue, double saturation)
{
    cv::Mat3f bgr;
    cv::Vec3f hsv;
    hsv[0] = (float)hue;
    hsv[1] = (float)255 * pow(saturation, 10);
    hsv[2] = (float)255;
    cvtColor(cv::Mat3f(3, 3, hsv), bgr, CV_HSV2BGR);
    return bgr.at<cv::Vec3f>(0, 0);
}
