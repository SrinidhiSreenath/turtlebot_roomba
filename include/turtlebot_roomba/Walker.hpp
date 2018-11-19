/************************************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2018, Srinidhi Sreenath
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    Walker.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    11/18/2018
 *  @version 1.0
 *
 *  @brief   Walker class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Walker which publishes navigation commands to a
 *  turtlebot based on laser scan data to emulate roomba robot behavior
 *
 */
#ifndef INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_
#define INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_

// ROS Headers
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 *  @brief Class Walker
 *
 *  The following class Walker publishes linear and angular velocity to a
 *  turtlebot based on laser scan data to demonstrate a simple walker algorithm
 */
class Walker {
 private:
  ros::NodeHandle n_;           ///< ROS nodehandle object
  ros::Subscriber laserSub_;    ///< ROS suscriber object
  ros::Publisher velocityPub_;  ///< ROS publisher object
  bool changeCourse_;  ///< bool variable to indicate whether the robot has to
                       ///< change its course while navigating
  geometry_msgs::Twist msg;  ///< message of twist type to publish linear x and
                             ///< angular z velocities

  /**
   *   @brief  function to process laser scan data
   *
   *   @param  scan as a pointer to laserscan message in sensor_msgs namespace
   *           containing information about the scan data from turtlebot
   *
   *   @return void
   */
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);

  /**
   *   @brief  function to publish navigation commands to turtlebot
   *
   *   @param  changeCourse as a bool to indicate change in course. If it's
   *           true. the robot has to change it's route to avoid obstacles and
   *           if it's false the robot keeps navigating in a straight line which
   *           is the default behavior.
   *
   *   @return void
   */
  void navigate(bool changeCourse);

 public:
  /**
   *   @brief  Default constructor for Walker. Sets up a publisher topic with
   *           ROS master and subscribes to laser scan data
   *
   *   @param  n as ROS nodehandle object
   *
   *   @return void
   */
  explicit Walker(ros::NodeHandle &n);

  /**
   *   @brief  Destructor for Walker
   *
   *   @param  none
   *
   *   @return void
   */
  ~Walker();
};

#endif  //   INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_
