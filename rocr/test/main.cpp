/**
 *  @copyright MIT License, © 2021 Vivek Sood, Markose Jacob, Yash Kulkarni
 *  @file    main.cpp
 *  @authors  Vivek Sood, Markose Jacob, Yash Kulkarni
 *  @date    12/06/2021
 *  @version 1.0
 *  @brief   Main file to run all tests
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}