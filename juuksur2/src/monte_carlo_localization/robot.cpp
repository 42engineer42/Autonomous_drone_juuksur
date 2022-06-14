//Standard libraries
#include <vector>
#include <stdexcept>
#include <math.h>
//Project libraries
#include "util.hpp"

//namespace
//{
#include "landmarks.hpp"
//}

//Header file
#include "robot.hpp"

Robot::Robot(double world_x, double world_y) : _x(Util::gen_real_random() * world_x),
                                               _y(Util::gen_real_random() * world_y),
                                               _orient(Util::gen_real_random() * 2.0 * M_PI),
                                               _forward_noise(0.0),
                                               _turn_noise(0.0),
                                               _sense_noise(0.0),
                                               _world_x(world_x),
                                               _world_y(world_y)
{
}

void Robot::set(double new_x, double new_y, double new_orient)
{
    // Set robot new position and orientation
    if (new_x < 0 || new_x >= _world_x)
    {
        throw std::invalid_argument("X coordinate out of bound: " + std::to_string(new_x));
    }
    if (new_y < 0 || new_y >= _world_y)
    {
        throw std::invalid_argument("Y coordinate out of bound: " + std::to_string(new_y));
    }
    if (new_orient < 0 || new_orient >= 2 * M_PI)
    {
        throw std::invalid_argument("Orientation must be in [0..2pi]");
    }
    _x = new_x;
    _y = new_y;
    _orient = new_orient;
}

void Robot::set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
{
    // Simulate noise, often useful in particle filters
    _forward_noise = new_forward_noise;
    _turn_noise = new_turn_noise;
    _sense_noise = new_sense_noise;
}

std::vector<double> Robot::sense() //TODO:Needs fixing
{
    // Measure the distances from the robot toward the landmarks
    std::vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
    double dist;

    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++)
    {
        dist = sqrt(pow((_x - landmarks[i][0]), 2) + pow((_y - landmarks[i][1]), 2));
        dist += Util::gen_gauss_random(0.0, _sense_noise);
        z[i] = dist;
    }
    return z;
}

std::vector<std::pair<double, double>> Robot::sense(double height)
{
    std::pair<double, double> rect = get_vision_rectangle(height);
    std::vector<std::pair<double, double>> measurements = Landmarks::get_landmarks_in_rect(rect, std::make_pair(_x, _y), _orient);
    return measurements;
}

Robot Robot::move(double turn, double forward)
{
    if (forward < 0)
        throw std::invalid_argument("Robot cannot move backward");

    // turn, and add randomness to the turning command
    _orient = _orient + turn + Util::gen_gauss_random(0.0, _turn_noise);
    _orient = Util::mod(_orient, 2 * M_PI);

    // move, and add randomness to the motion command
    double dist = forward + Util::gen_gauss_random(0.0, _forward_noise);
    _x = _x + (cos(_orient) * dist);
    _y = _y + (sin(_orient) * dist);

    _x = Util::mod(_x, _world_x);
    _y = Util::mod(_y, _world_y);

    // set particle
    Robot res(_world_x, _world_y);
    res.set(_x, _y, _orient);
    res.set_noise(_forward_noise, _turn_noise, _sense_noise);

    return res;
}

std::string Robot::show_pose()
{
    // Returns the robot current position and orientation in a string format
    return "[x=" + std::to_string(_x) + " y=" + std::to_string(_y) + " orient=" + std::to_string(_orient) + "]";
}

double Robot::get_x()
{
    return this->_x;
}

double Robot::get_y()
{
    return this->_y;
}

double Robot::get_orient()
{
    return this->_orient;
}

std::string Robot::read_sensors() //UNUSED
{
    // Returns all the distances from the robot toward the landmarks
    std::vector<double> z = sense();
    std::string readings = "[";
    for (int i = 0; i < z.size(); i++)
    {
        readings += std::to_string(z[i]) + " ";
    }
    readings[readings.size() - 1] = ']';

    return readings;
}

double Robot::measurement_prob(std::vector<double> measurement) //Soon to be deprecated
{
    // Calculates how likely a measurement should be
    double prob = 1.0;
    double dist;

    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++)
    {
        dist = sqrt(pow((_x - landmarks[i][0]), 2) + pow((_y - landmarks[i][1]), 2));
        prob *= gaussian(dist, _sense_noise, measurement[i]);
    }

    return prob;
}

double Robot::measurement_prob(std::vector<std::pair<double, double>> measurements)
{
    double prob = 1.0;

    for (auto measurement : measurements)
    {
        double distance = Landmarks::get_distance_squared_to_closest(Landmarks::transform_to_point(measurement, std::make_pair(_x, _y), _orient));
        prob *= zero_mean_gaussian(SIGMA, distance);//gaussian(0, _sense_noise, distance);
    }

    return prob;
}

double Robot::get_world_x()
{
    return _world_x;
}
double Robot::get_world_y()
{
    return _world_y;
}

////////////////////////////////////////////

double Robot::gaussian(double mu, double sigma, double x)
{
    // Probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(-(pow(((mu - x) / sigma), 2)) / 2.0) / (sqrt(2.0 * M_PI) * sigma);
}

double Robot::zero_mean_gaussian(double sigma, double x)
{
    // Probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(-pow(x / sigma, 2)); //exp(-(pow(x / sigma, 2)) / 2.0) / (sqrt(2.0 * M_PI) * sigma);
}

std::pair<double, double> Robot::get_vision_rectangle(double height)
{
    return std::make_pair(2 * tan(X_CAMERA_ANGLE / 2) * height,
                          2 * tan(Y_CAMERA_ANGLE / 2) * height);
}