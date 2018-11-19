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
 *  @file    main.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    11/17/2018
 *  @version 1.0
 *
 *  @brief   Main source file
 *
 *  @section DESCRIPTION
 *
 *  Main source file to implement the walker class for navigation of turtlebot.
 *
 */
// ROS Console
#include <ros/console.h>

// Walker class header
#include "turtlebot_roomba/Walker.hpp"

/**
 *   @brief  Main function to set up the ROS node and implement the navigation
 *           for turtlebot.
 *
 *   @param  argc - argument count as integer
 *   @param  argv - argument vector as character array
 *
 *   @return integer 0 indication successful execution
 */
int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "turtlebot_walker");
  ros::NodeHandle nh;

  // Change logger level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Construct the walker class for navigation of turtlebot
  Walker walker(nh);

  // Spin node until termination
  ros::spin();

  return 0;
}
