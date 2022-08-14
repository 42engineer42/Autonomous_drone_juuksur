//Header file
#include "landmarks.hpp"
#include <math.h>
#include <limits>

std::vector<std::pair<double, double>> Landmarks::get_landmarks_in_rect(std::pair<double, double> rect, std::pair<double, double> loc, double theta)
{
    std::vector<std::pair<double, double>> output;
    double rect_x_max = rect.first / 2;
    double rect_x_min = -rect.first / 2;
    double rect_y_max = rect.second / 2;
    double rect_y_min = -rect.second / 2;
    for (auto landmark : landmarks)
    {
        std::pair<double, double> transformed_point = Landmarks::transform_point(landmark, loc, theta);
        if (transformed_point.first < rect_x_max && transformed_point.first > rect_x_min && transformed_point.second < rect_y_max && transformed_point.second > rect_y_min)
        {
            output.push_back(transformed_point);
        }
    }
    return output;
}

std::pair<double, double> Landmarks::transform_to_point(std::pair<double, double> point, std::pair<double, double> loc, double theta)
{
    double x = point.first * cos(-theta) - point.second * sin(-theta);
    double y = point.second * cos(-theta) + point.first * sin(-theta);
    return std::make_pair(x + loc.first,
                          y + loc.second);
}

std::pair<double, double> Landmarks::transform_point(std::pair<double, double> point, std::pair<double, double> loc, double theta)
{
    /*
    double x = point.first * cos(theta) - point.second * sin(theta);
    double y = point.second * cos(theta) + point.first * sin(theta);
    return std::make_pair(x - loc.first,
                          y - loc.second);
    */
    double x = point.first - loc.first;
    double y = point.second - loc.second;
    return std::make_pair(x * cos(theta) - y * sin(theta),
                          y * cos(theta) + x * sin(theta));
}

std::pair<double, double> Landmarks::transform_point(double *point, std::pair<double, double> loc, double theta)
{
    double x = point[0] - loc.first;
    double y = point[1] - loc.second;
    return std::make_pair(x * cos(theta) - y * sin(theta),
                          y * cos(theta) + x * sin(theta));
}

double Landmarks::get_distance_squared_to_closest(std::pair<double, double> point)
{
    double best = std::numeric_limits<double>::max();
    for (auto landmark : landmarks)
    {
        double distance_squared = pow(landmark[0] - point.first, 2) + pow(landmark[1] - point.second, 2);
        if (distance_squared < best)
            best = distance_squared;
    }
    return best;
}
