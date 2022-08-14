#ifndef ROBOT_HPP
#define ROBOT_HPP
//Standard libraries
#include <vector>
#include <string>

class Robot
{
public:
    Robot(double world_x, double world_y);
    void set(double new_x, double new_y, double new_orient);
    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);
    std::vector<double> sense(); //Soon to be deprecated
    std::vector<std::pair<double, double>> sense(double height);
    Robot move(double turn, double forward);
    std::string show_pose(); //DEPRECATED
    double get_x();
    double get_y();
    double get_orient();
    double get_world_x();
    double get_world_y();
    std::string read_sensors();                               //DEPRECATED
    double measurement_prob(std::vector<double> measurement); //Soon to be deprecated
    double measurement_prob(std::vector<std::pair<double, double>> measurements);

private:
    double _x, _y, _orient;                           //robot poses
    double _forward_noise, _turn_noise, _sense_noise; //robot noises
    double _world_x, _world_y;
    static constexpr double X_CAMERA_ANGLE = 1.086;
    static constexpr double Y_CAMERA_ANGLE = 0.852;
    static constexpr double NOT_SEEN_PROBABILITY = 0.2; //Might be necessary
    static constexpr double SIGMA = 0.1;
    double gaussian(double mu, double sigma, double x);
    double zero_mean_gaussian(double sigma, double x);
    std::pair<double, double> get_vision_rectangle(double height);
};

#endif