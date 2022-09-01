#ifndef UTIL_HPP
#define UTIL_HPP
//Standard libraries
#include <random>
//Project libraries
#include "robot.hpp"

// Global Functions
namespace Util
{
// Random Generators
static std::random_device rd;
static std::mt19937 gen(rd());
double mod(double first_term, double second_term);
double gen_real_random();
double evaluation(Robot r, std::vector<Robot> p, int n);
double gen_gauss_random(double mean, double variance);
double max(double arr[], int n);
double find_median(double a[], int n);
} // namespace Util
#endif