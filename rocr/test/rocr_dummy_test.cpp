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



/**
*   @brief A test to ensure that the test setup works
*/
TEST(rocr, dummy_test) {
    std::string dummy = "Checking Test Setup";
    EXPECT_EQ( "Checking Test Setup", "Checking Test Setup");
}