//Standard libraries
#include <random>
#include <algorithm>
#include <iterator>
//Project libraries
#include "robot.hpp"
//Header file
#include "util.hpp"

// Functions
double Util::gen_real_random()
{
    // Generate real random between 0 and 1
    std::uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    //std::random_device rd;
    //std::mt19937 gen(rd());
    return real_dist(gen);
}

double Util::mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double Util::evaluation(Robot r, std::vector<Robot> p, int n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        //the second part is because of world's cyclicity
        double dx = mod((p[i].get_x() - r.get_x() + (r.get_world_x() / 2.0)), r.get_world_x()) - (r.get_world_x() / 2.0);
        double dy = mod((p[i].get_y() - r.get_y() + (r.get_world_y() / 2.0)), r.get_world_y()) - (r.get_world_y() / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}

double Util::gen_gauss_random(double mean, double variance)
{
    // Gaussian random
    std::normal_distribution<double> gauss_dist(mean, variance);
    //std::random_device rd;
    //std::mt19937 gen(rd());
    return gauss_dist(gen);
}

double Util::max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++)
    {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

double Util::find_median(double a[], int n)
{
    // First we sort the array
    double b[n];
    std::copy(a, a + n, b);

    std::sort(b, b + n);

    // check for even case
    if (n % 2 != 0)
        return (double)b[n / 2];

    return (double)(b[(n - 1) / 2] + b[n / 2]) / 2.0;
}