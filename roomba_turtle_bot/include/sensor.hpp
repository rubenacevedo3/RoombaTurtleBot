/**
 *@author Ruben Acevedo
 *@file sensor.hpp
 *@brief This is the ".hpp" file for the sensor Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file will define the methods and attributes of the
 * sensor Class
 */
/**
 * MIT License
 *
 * Copyright 2017 Ruben Acevedo
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software. THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. © 2017 GitHub, Inc.
 */

#ifndef CATKIN_WS_SRC_ROOMBATURTLEBOT_ROOMBA_TURTLE_BOT_INCLUDE_SENSOR_HPP_
#define CATKIN_WS_SRC_ROOMBATURTLEBOT_ROOMBA_TURTLE_BOT_INCLUDE_SENSOR_HPP_

#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"

//! sensor Class
/**
 * This class gits the minimum distance the turtle bot senses
 */
class sensor {
  //! Public Methods
 public:
  //! Class Constructor
  /**
   * @brief This code constructs the class.
   * It initializes the min distance to be 1000
   * @param nothing
   * @return nothing
   */
  sensor();

  //! in collison function
  /**
   * @brief checks to see if the robot is in collison 
   * @param nothing
   * @return a bool stating whether or not a command was matched
   */
  bool inCollision();

  //! get min distance function
  /**
   * @brief it gets the min distance from the laser
   * @param a constant string reference representing a command name
   * @param a constant string reference representing a microphone signal
   * @return a bool stating whether or not a command was trained
   */
  void getMinDistanceCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  //! Private Attributes
 private:
  //! min ditance
  /**
   * @brief its the min distance from the laser
   */
  float minDist;
};

#endif  // CATKIN_WS_SRC_ROOMBATURTLEBOT_ROOMBA_TURTLE_BOT_INCLUDE_SENSOR_HPP_
