/**
 *  @copyright MIT License, © 2021 Vivek Sood, Markose Jacob, Yash Kulkarni
 *  @file    rocr_dummy_test.cpp
 *  @authors  Vivek Sood, Markose Jacob, Yash Kulkarni
 *  @date    12/06/2021
 *  @version 1.0
 *  @brief   Dummy Test  file to check the test setup
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include "rocr/teleop.hpp"


/**
*   @brief A test to ensure that the teleop works
*/
TEST(rocr_test, test) {
    Teleop teleop;
    ros::NodeHandle n;

    EXPECT_EQ(teleop.get_moveBindings()['i'][0], 1);
    EXPECT_EQ(teleop.get_speedBindings()['q'][0], 1.1);
    EXPECT_EQ(teleop.get_moveBindings()[1][0], 0.0);
}