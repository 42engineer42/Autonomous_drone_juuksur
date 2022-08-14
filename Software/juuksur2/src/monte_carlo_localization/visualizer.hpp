#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP
// Standard libraries
#include <vector>
#include <utility>
#include <string>
// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Visualizer
{
public:
    Visualizer(double max_x, double max_y, double pixels_per_meter);
    void draw_points(std::vector<std::pair<double, double>> points, int color);
    void draw_points(std::vector<std::pair<std::pair<double, double>, double>> points, int color);
    void draw_points(std::vector<std::pair<std::pair<double, double>, double>> points, int color, double saturations[]);
    void write_image(std::string tag);
private:
    cv::Mat _image;
    double _max_x, _max_y;
    double _pixels_per_meter;
    cv::Scalar get_bgr(int hsv);
    cv::Scalar get_secondary_bgr(int hue, int value);
    cv::Scalar get_weighted_bgr(int hue, double saturation);
};

#endif