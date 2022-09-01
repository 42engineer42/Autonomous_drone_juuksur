// Standard libraries
#include "math.h"
// External libraries
#include <yaml-cpp/yaml.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
// Framework libraries
#include "ros/ros.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// Framework messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
// SmartDrone messages
#include "dev_msgs/MoveData.h"

#include "ttmath.h"

int image_width = 0;
int image_height = 0;
double min_contour_area = 300;
double X_CAMERA_ANGLE = 1.086; // horizontal fov radians
double Y_CAMERA_ANGLE = 0.852; // vertical fov radians
int PITCH_VALUE = 20;
int ROLL_VALUE = 20;

double height = 0;
double height_coefficient = 1;

int pitch, roll;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage out_msg;
image_transport::Publisher debug_image_pub;
//ros::Publisher debug_image_pub;

std::string type2str(int type)
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth)
  {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

void white_balance(cv::Mat& image)
{
  // std::cout << type2str(image.type()) << "\n";
  image.convertTo(image, CV_32FC1, 1.0 / 255.0);
  // std::cout << type2str(image.type()) << "\n";
  cv::pow(image, 2, image);
  double min, max;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(image, &min, &max, &min_loc, &max_loc);
  double multiplier = 1.0 / max;
  cv::multiply(image, multiplier, image);
}

std::vector<std::pair<int, int>> get_centers(cv::Mat& image)
{
  image.convertTo(image, CV_8UC1, 1.0);
  cv::threshold(image, image, 50, 255, CV_THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  std::cout << "contours: " << contours.size() << "\n";
  cv::cvtColor(image, image, CV_GRAY2BGR);
  // std::cout << type2str(image.type()) << "\n";

  std::vector<std::pair<int, int>> output;
  // cv::drawContours(image, contours, -1, cv::Scalar(0, 0, 255), 2);
  for (int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]);
    // std::cout << "area: " << area << "\n";
    if (min_contour_area < area)
    {
      cv::Moments M = cv::moments(contours[i]);

      if (M.m00 != 0)
      {
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        output.push_back(std::make_pair(cx, cy));

        cv::ellipse(image, cv::RotatedRect(cv::Point(cx, cy), cv::Size(3, 3), 0), cv::Scalar(255, 0, 0), 3);
      }
    }
  }
  // std::cout << "\n";
  return output;
}

std::vector<std::pair<double, double>> get_locations(std::vector<std::pair<int, int>> centers)
{
  std::vector<std::pair<double, double>> output;
  for (auto center : centers)
  {
    // x on floor and y on floow relative to view center
    output.push_back(std::make_pair(height * tan((center.first - image_width / 2) * X_CAMERA_ANGLE / image_width),
                                    height * tan((center.second - image_height / 2) * Y_CAMERA_ANGLE / image_height)));
  }
  return output;
}

long calculate_distance_squared(std::pair<double, double> a, std::pair<double, double> b)
{
  return pow(a.first - b.first, 2) + pow(a.second - b.second, 2);
}

bool is_same(std::pair<double, double> a, std::pair<double, double> b)
{
  return a.first == b.first && a.second == b.second;
}

std::vector<std::pair<double, double>> sort_objects(std::vector<std::pair<double, double>> locations)
{
  std::vector<std::vector<long>> distances;
  std::deque<int> order;

  for (int i = 0; i < locations.size(); i++)
  {
    distances.push_back(std::vector<long>());
    for (int j = 0; j < locations.size(); j++)
    {
      if (i == j)
        distances[i].push_back(LONG_MAX);
      else
        distances[i].push_back(calculate_distance_squared(locations[i], locations[j]));
    }
  }

  if (locations.size() > 0)
  {
    order.push_back(0);
  }

  while (order.size() < locations.size())
  {
    // int front_best = std::min_element(distances[order.front()].begin(), distances[order.front()].end()) -
    // distances[order.front()].begin();
    int best_index;
    long best_distance = LONG_MAX;
    for (int i = 0; i < locations.size(); i++)
    {
      long distance = distances[i][order.front()];
      if (std::find(order.begin(), order.end(), i) == order.end() && distance < best_distance)
      {
        best_distance = distance;
        best_index = i;
      }
    }
    if (order.front() != order.back())
    {
      int best_index_back;
      long best_distance_back = LONG_MAX;
      for (int i = 0; i < locations.size(); i++)
      {
        long distance = distances[i][order.back()];
        if (std::find(order.begin(), order.end(), i) == order.end() && distance < best_distance_back)
        {
          best_distance_back = distance;
          best_index_back = i;
        }
      }
      if (best_distance_back < best_distance)
      {
        order.push_back(best_index_back);
      }
      else
      {
        order.push_front(best_index);
      }
    }
    else
    {
      order.push_front(best_index);
    }
  }

  std::vector<std::pair<double, double>> output;
  for (auto it = order.cbegin(); it != order.cend(); ++it)
    output.push_back(locations[*it]);

  return output;
}

sensor_msgs::ImagePtr to_image_msg(cv::Mat image)
{
  out_msg.image = image;
  return out_msg.toImageMsg();
}

void follow_objects(std::vector<std::pair<double, double>> locations)
{
  cv::Mat trajectory(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));
  locations = sort_objects(locations);
  std::pair<double, double> closest;
  long best_distance = LONG_MAX;
  for (int i = 0; i < locations.size(); i++)
  {
    long distance = calculate_distance_squared(std::make_pair(0, 0), locations[i]);
    if (distance < best_distance)
    {
      best_distance = distance;
      closest = locations[i];
    }
  }
  for (int i = 0; i < locations.size() - 1; i++)
  {
    if (locations.size() == 0)
      break;  // SOMETHING FUNNY GOING ON HERE
    std::cout << "sorted loc: " << locations[i].first << " : " << locations[i].second << "\n";
    std::cout << "coords: " << (int)(locations[i].first * 100 + 100) << " : " << (int)(locations[i].second * 100 + 100)
              << "\n";
    cv::line(trajectory, cv::Point((int)(locations[i].first * 100 + 100), (int)(locations[i].second * 100 + 100)),
             cv::Point((int)(locations[i + 1].first * 100 + 100), (int)(locations[i + 1].second * 100 + 100)),
             cv::Scalar(0, 0, 255));
  }
  std::cout << "\n";

  if (best_distance < LONG_MAX)
  {
    pitch = closest.first < 0 ? -PITCH_VALUE : closest.first > 0 ? PITCH_VALUE : 0;
    roll = closest.second < 0 ? ROLL_VALUE : closest.second > 0 ? -ROLL_VALUE : 0;
    
    pitch += 1500;
    roll += 1500;
    std::cout << "pitch: " << pitch << " roll: " << roll << "\n";
  }

   //cv::imshow("trajectory", trajectory);
   //cv::waitKey(1);
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    printf("img callback!\r\n");
  // ROS_INFO("HELLO");
  cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
  // ROS_INFO("%d", cv_ptr_rgb->header.seq);
  cv::Mat image = cv_ptr_rgb->image;
  //cv::imshow("image", image);
  //cv::waitKey(1);
  cv::cvtColor(image, image, CV_BGR2GRAY);
  image_width = image.cols;
  image_height = image.rows;
  //white_balance(image);
  std::vector<std::pair<int, int>> centers = get_centers(image);
  std::vector<std::pair<double, double>> locations = get_locations(centers);
  for (auto location : locations)
  {
    std::cout << "location: " << location.first << " : " << location.second << "\n";
  }
  std::cout << "\n";
  // std::cout << type2str(image.type()) << "\n";

  cv::Point pitch_point(image_width / 2, image_height / 2);
  if(pitch > 1500)pitch_point = cv::Point(image_width / 2 + 100, image_height / 2);
  else if(pitch < 1500)pitch_point = cv::Point(image_width / 2 - 100, image_height / 2);
  cv::line(image, cv::Point(image_width / 2, image_height / 2), pitch_point, cv::Scalar(0, 255, 0));

  cv::Point roll_point(image_width / 2, image_height / 2);
  if(roll < 1500)roll_point = cv::Point(image_width / 2, image_height / 2 + 100);
  else if(roll > 1500)roll_point = cv::Point(image_width / 2, image_height / 2 - 100);
  cv::line(image, cv::Point(image_width / 2, image_height / 2), roll_point, cv::Scalar(0, 0, 255));


  cv::resize(image, image, cv::Size(640, 480));
  debug_image_pub.publish(to_image_msg(image));
  cv::imshow("image", image);
  cv::waitKey(1);
  follow_objects(locations);

  // cv::waitKey(1);
}

void height_callback(std_msgs::Float32 msg)
{
  height = msg.data * height_coefficient;
}

int main(int argc, char** argv)
{
  roll = 1500;
  pitch = 1500;
  ros::init(argc, argv, "camera_recognition");
  ros::NodeHandle n;
  YAML::Node nodeConf = YAML::LoadFile(ros::package::getPath("camera_recognition") + "/config/node.yaml");

  ros::Subscriber image_subscriber = n.subscribe(nodeConf["topics"]["image"].as<std::string>(), 1, image_callback);
  ros::Subscriber height_subscriber = n.subscribe(nodeConf["topics"]["height"].as<std::string>(), 1, height_callback);
  height_coefficient = nodeConf["height"]["coefficient"].as<double>();

  ros::Publisher pitchPub = n.advertise<std_msgs::UInt16>("/control/pitch", 1, true);
  ros::Publisher rollPub = n.advertise<std_msgs::UInt16>("/control/roll", 1, true);
  //debug_image_pub = n.advertise<sensor_msgs::Image>("/image/debug", 1, true);
  image_transport::ImageTransport it(n);
  debug_image_pub = it.advertise("/image/debug", 1);
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;

  std_msgs::UInt16 v;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    v.data = roll;
    rollPub.publish(v);
    v.data = pitch;
    pitchPub.publish(v);

    loop_rate.sleep();
  }
}
