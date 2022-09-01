#ifndef LANDMARKS_HPP
#define LANDMARKS_HPP
//Standard libraries
#include <vector>
#include <utility>

static double landmarks[47][2] =
    {{5.0, 2.493}, {5.329, 2.28}, {5.689, 2.082}, {6.078, 1.898}, {6.437, 1.686}, {6.796, 1.473}, {7.201, 1.289}, {7.605, 1.204}, {8.024, 1.261}, {8.398, 1.416}, {8.713, 1.7}, {8.907, 2.054}, {8.982, 2.45}, {8.922, 2.861}, {8.713, 3.215}, {8.443, 3.499}, {8.054, 3.683}, {7.65, 3.739}, {7.231, 3.669}, {6.841, 3.484}, {6.467, 3.286}, {6.093, 3.088}, {5.734, 2.875}, {5.374, 2.691}, {4.626, 2.28}, {4.192, 2.096}, {3.877, 1.87}, {3.518, 1.671}, {3.144, 1.473}, {2.769, 1.289}, {2.365, 1.204}, {1.946, 1.261}, {1.557, 1.431}, {1.257, 1.7}, {1.048, 2.054}, {0.988, 2.45}, {1.048, 2.833}, {1.228, 3.187}, {1.512, 3.47}, {1.901, 3.669}, {2.32, 3.739}, {2.754, 3.669}, {3.129, 3.499}, {3.488, 3.3}, {3.847, 3.102}, {4.237, 2.89}, {4.611, 2.691}};

namespace Landmarks
{
std::vector<std::pair<double, double>> get_landmarks_in_rect(std::pair<double, double> rect, std::pair<double, double> loc, double theta);
std::pair<double, double> transform_point(std::pair<double, double> point, std::pair<double, double> loc, double theta);
std::pair<double, double> transform_point(double *point, std::pair<double, double> loc, double theta);
std::pair<double, double> transform_to_point(std::pair<double, double> point, std::pair<double, double> loc, double theta);
double get_distance_squared_to_closest(std::pair<double, double> point);
} // namespace Landmarks

#endif