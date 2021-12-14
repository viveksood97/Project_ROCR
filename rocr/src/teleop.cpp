/**
 *  @Copyright 2021 Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @file teleop.cpp
 *  @author Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @date 12/12/2021
 *
 *  @brief Teleop module
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
 * 
 * 
  */


#include <rocr/teleop.hpp>

Teleop::Teleop() {}
Teleop::~Teleop() {}

int Teleop::getch() {
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

std::map<char, std::vector<float>> Teleop::get_moveBindings() {
  return moveBindings;
}

std::map<char, std::vector<float>> Teleop::get_speedBindings() {
  return speedBindings;
}
std::map<char, std::vector<float>> Teleop::get_armBindings() {
  return armBindings;
}



