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
 *  @file    Walker.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    11/18/2018
 *  @version 1.0
 *
 *  @brief   Walker class definition.
 *
 *  @section DESCRIPTION
 *
 *  Source file for class Walker. The class implements a simple walker algorithm
 *  to emulate roombs robot's behavior. The class commands the turtlebot to
 *  navigate and subscribes to laser scan data to check for potential
 *  collisions. If a possible collision is expected, the turtlebot is asked to
 *  navigate in a different direction to avoid collision.
 *
 */

// Walker class header
#include "turtlebot_roomba/Walker.hpp"

// CPP Headers
#include <cmath>
#include <limits>
#include <vector>

Walker::Walker(ros::NodeHandle &n) : n_(n) {
  ROS_INFO_STREAM("Walker Node Initialized!");
  // Setup the velocity publisher topic with master
  velocityPub_ =
      n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  // subscribe to the laserscan topic
  laserSub_ = n_.subscribe("scan", 50, &Walker::processLaserScan, this);
}

Walker::~Walker() {}

void Walker::processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
  // Get min and max range values
  auto minRange = scan->range_min;
  auto maxRange = scan->range_max;

  // Get angle resolution
  auto angleRes = scan->angle_increment;

  // Extract scan range values
  std::vector<float> rangeValues = scan->ranges;
  size_t size = rangeValues.size();
  auto mid = size / 2;

  // Check for obstacle within a range of 20 degrees from center i.e 10 degrees
  // on left and 10 degrees on the right of center
  float checkRange = 10 * M_PI / 180;
  size_t checkRangeIncrement = checkRange / angleRes;

  // Get minimum range value in that region of interest
  float minDist = std::numeric_limits<float>::max();
  for (size_t it = mid - checkRangeIncrement; it < mid + checkRangeIncrement;
       it++) {
    if (rangeValues[it] < minDist) {
      minDist = rangeValues[it];
    }
  }
  // ROS_DEBUG_STREAM("Minimum distance: " << minDist);

  // If the scan value is invalid, then there is no change in turtlebot's
  // behavior i.e keep going straight.
  if (std::isnan(minDist) || minDist > maxRange || minDist < minRange) {
    changeCourse_ = false;
    navigate(changeCourse_);
    return;
  }

  // If minimum distance is close i.e 1 meter, command the turtlebot to rotate
  // else there is no change in course
  if (minDist < 1.0) {
    ROS_WARN_STREAM("Obstacle detected ahead! Changing course");
    changeCourse_ = true;
  } else {
    changeCourse_ = false;
  }
  // Publish navigation command to turtlebot
  navigate(changeCourse_);
}

void Walker::navigate(bool changeCourse) {
  // the default behavior is to keep going straight. If change in course is
  // requested, the turtlebot is commanded to rotate
  if (changeCourse) {
    msg.linear.x = 0.0;
    msg.angular.z = -0.5;
  } else {
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;
  }

  // Publish the desired velocity commands
  velocityPub_.publish(msg);
}
