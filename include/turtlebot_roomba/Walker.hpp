#ifndef INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_
#define INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class Walker {
 private:
  ros::NodeHandle n_;
  ros::Subscriber laserSub_;
  ros::Publisher velocityPub_;
  bool changeCourse_;
  geometry_msgs::Twist msg;

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);
  void navigate(bool changeCourse);

 public:
  explicit Walker(ros::NodeHandle &n);
  ~Walker();
};

#endif  //   INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_
