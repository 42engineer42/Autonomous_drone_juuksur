//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
//#include "src/matplotlibcpp.h" //Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random>    //C++ 11 Random Numbers
#include <vector>
//Project includes
#include "visualizer.hpp"
#include "robot.hpp"

#include "util.hpp"

//namespace {
#include "landmarks.hpp"
//}
//namespace plt = matplotlibcpp;
using namespace std;

// Landmarks

// Map size in meters
//double world_size = 10.0;
double world_size_x = 10.0;
double world_size_y = 5.0;

double start_x = 1;
double start_y = 2.5;
double start_box = 1;

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot(world_size_x, world_size_y);
    myrobot.set_noise(1.0, 0.1, 5.0);
    myrobot.set(0, 0, M_PI / 2.0);

    // Create a set of particles
    int n = 1000;
    std::vector<Robot> p;
    for (int i = 0; i < n; i++)
    {
        p.push_back(Robot(world_size_x, world_size_y));
        /*p[i].set((Util::gen_real_random() - 0.5) * start_box + start_x,
                 (Util::gen_real_random() - 0.5) * start_box + start_y,
                 Util::gen_real_random() * 2.0 * M_PI);*/
    }
    //Robot p[n](world_size_x, world_size_y);

    for (int i = 0; i < n; i++)
    {
        p[i].set_noise(0.25, 0.25, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot(world_size_x, world_size_y);
    myrobot.set(1, 2.5, M_PI * 3 / 2.0);
    //vector<double> z;

    //Iterating 50 times over the set of particles
    int steps = 1500;
    for (int t = 0; t < steps; t++)
    {

        Visualizer v(world_size_y, world_size_x, 100);
        //Move the robot and sense the environment afterwards
        myrobot = myrobot.move(0.03, 0.05);
        vector<pair<double, double>> z = myrobot.sense(1.5);
        //vector<double> z = myrobot.sense();

        // Simulate a robot motion for each of these particles
        std::vector<Robot> p2(n, Robot(world_size_x, world_size_y));
        for (int i = 0; i < n; i++)
        {
            p2[i] = p[i].move(0.03 * Util::gen_real_random() * 2, 0.05);
            p[i] = p2[i];
        }

        //Generate particle weights depending on robot's measurement
        double w[n];
        for (int i = 0; i < n; i++)
        {
            w[i] = p[i].measurement_prob(z);
        }
        double mw = Util::max(w, n);
        double w_viz[n];
        for (int i = 0; i < n; i++)
        {
            w_viz[i] = w[i] / mw;
        }

        //VISUALIZATION
        vector<std::pair<std::pair<double, double>, double>> robots_vec;
        for (int i = 0; i < n; i++)
        {
            robots_vec.push_back(std::make_pair(std::make_pair(p[i].get_x(), p[i].get_y()), p[i].get_orient()));
        }

        v.draw_points(robots_vec, 70, w_viz);
        v.draw_points(std::vector<std::pair<std::pair<double, double>, double>>{std::make_pair(std::make_pair(myrobot.get_x(),
                                                                                                              myrobot.get_y()),
                                                                                               myrobot.get_orient())},
                      240);

        vector<std::pair<double, double>> landmarks_vec;
        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++)
        {
            landmarks_vec.push_back(std::make_pair(landmarks[i][0], landmarks[i][1]));
        }
        v.draw_points(landmarks_vec, 0);
        vector<pair<double, double>> z_viz;
        for(auto point : z){
            z_viz.push_back(Landmarks::transform_to_point(point, std::make_pair(myrobot.get_x(), myrobot.get_y()), myrobot.get_orient()));
        }
        v.draw_points(z_viz, 60);
        v.write_image(std::to_string(t));

        //std::cout << "s W: ";
        //for(int i=0; i<n; i++)std::cout << w[i] << " ";
        //std::cout << "\n";
        ///////////////////////////////////////
        /*
        double median = Util::find_median(w, n);
        std::cout << "median: " << median << "\n";
        int counter = 0;
        for (int i = 0; i < n; i++)
        {
            if (w[i] < median)
            {
                p[i] = Robot(world_size_x, world_size_y);
                ++counter;
            }
        }
        //std::cout << "e W: ";
        //for(int i=0; i<n; i++)std::cout << w[i] << " ";
        //std::cout << "\n";
        std::cout << "deleted: " << counter << "\n";
        */
        /////////////////
        //Resample the particles with a sample probability proportional to the importance weight
        
        std::vector<Robot> p3(n, Robot(world_size_x, world_size_y));
        int index = Util::gen_real_random() * n;
        double beta = 0.0;

        std::cout << "MW: " << mw << "\n";
        for (int i = 0; i < n; i++)
        {
            beta += Util::gen_real_random() * 2.0 * mw;
            while (beta > w[index])
            {
                beta -= w[index];
                index = Util::mod((index + 1), n);
            }
            p3[i] = p[index];
        }
        for (int k = 0; k < n; k++)
        {
            p[k] = p3[k];
        }
        
        //Evaluate the Error
        cout << "Step = " << t << ", Evaluation = " << Util::evaluation(myrobot, p, n) << endl;

    } //End of Steps loop

    return 0;
}
