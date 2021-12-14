/**
 *  @Copyright 2021 Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @file teleop.cpp
 *  @author Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @date 12/12/2021
 *
 *  @brief Teleop to move the robot and to open and close arms.
 *  @brief Original open source code taken from git clone https://github.com/methylDragon/teleop_twist_keyboard_cpp.git
 *
 *  @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2021 Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *  @section DESCRIPTION
 *
 *  Teleop for ROCR bot
 *
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <algorithm>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings {
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings {
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Create map armBindings
std::map<char, std::vector<float>> armBindings {
  {'1', {0.0, 0}},
  {'2', {-0.1, 0}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Opening Arm Controller
--------------------------------
Press 2 to open the arms
Press 1 to close the arms

CTRL-C to quit

)";
/**
 * @brief Init Variables
 * @param speed Linear velocity (m/s)
 * @param turn Angular velocity (rad/s)
 * @param x Forward direction
 * @param y backward direction
 * @param z neutral direction
 * @param th angle direction
 * @param arm_movements Arm Movement
 * @param key To read the key entered
 */
float speed(0.5);
float turn(1.0);
float x(0), y(0), z(0), th(0);
float arm_movement(0);
char key(' ');

/**
 * @brief For non-blocking keyboard inputs
 * 
 * @return int 
 */
int getch(void) {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv) {
  /**
   * @brief Init ROS node
   * 
   */
  ros::init(argc, argv, "teleop_twist_keyboard");
  /**
   * 
   * @param nh ROS NodeHandle
   */
  ros::NodeHandle nh;
  /**
   * @brief Publisher for cmd_vel topic to rotate wheels and move arms
   * @param pub cmd_vel publisher
   * @param pub_right_arm cmd_vel publisher
   * @param pub_left_arm cmd_vel publisher
   */
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>
  ("/rocr_controller/cmd_vel", 1);
  ros::Publisher pub_right_arm = nh.advertise<std_msgs::Float64>
  ("/controller_right_arm/command", 10);
  ros::Publisher pub_left_arm = nh.advertise<std_msgs::Float64>
  ("/controller_left_arm/command", 10);

  /**
   * @brief Creating Twist and Float messages for wheels and arms respectivly
   * @param twist for wheels
   * @param arm_control for left arm
   * @param arm_control_2 for right arm
   */
  geometry_msgs::Twist twist;
  std_msgs::Float64 arm_control, arm_control_2;

  printf("%s", msg);
  printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

  /**
   * @brief While loop which ends only when the program is terminated.
   * @brief Keep taking in put from the user and compares it with the key map
   * @brief Appropriate action is taken based on user choice
   */
  while (true) {
    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1) {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ",
      speed, turn, key);
      // Otherwise if it corresponds to a key in speedBindings
    } else if (speedBindings.count(key) == 1) {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ",
      speed, turn, key);
      // Otherwise if it corresponds to a key in armBindings
    } else if (armBindings.count(key) == 1) {
      // Grab arm movement
      arm_movement = armBindings[key][0];
      // Otherwise, set the robot to stop
    } else {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      arm_movement = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03') {
        break;
      }

      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c",
      speed, turn, key);
    }

    // Update the Twist message
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    arm_control.data = arm_movement;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    arm_control_2.data = -1 * arm_control.data;
    pub_right_arm.publish(arm_control);
    pub_left_arm.publish(arm_control_2);
    ros::spinOnce();
  }

  return 0;
}
