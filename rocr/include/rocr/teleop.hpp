/**
 *  @Copyright 2021 Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @file teleop.hpp
 *  @author Markose Jacob, Yash Mandar Kulkarni, Vivek Sood
 *  @date 12/12/2021
 *
 *  @brief Header file for teleop module
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
 *  Header for teleop for ROCR bot
 *
 *
 *
 */

#ifndef INCLUDE_DVD_ROBOT_TELEOP_HPP_
#define INCLUDE_DVD_ROBOT_TELEOP_HPP_

#include <unistd.h>
#include <termios.h>
#include <map>
#include <vector>

class Teleop {
public:
    /**
     * @brief Contructor for Teleop Module
     * 
     * @return void 
     */
    Teleop();
    /**
     * @brief Destructor for Teleop Module
     * 
     * @return void 
     */
    ~Teleop();
    /**
     * @brief For non-blocking keyboard inputs
     *
     * @return int
     */
    int getch(void);
    /**
     * @brief Getter for move bindings
     *
     * @return std::map<char, std::vector<float>>
     */
    std::map<char, std::vector<float>> get_moveBindings();
    /**
     * @brief Getter for speed bindings
     *
     * @return std::map<char, std::vector<float>>
     */
    std::map<char, std::vector<float>> get_speedBindings();
    /**
     * @brief Getter for arm bindings
     *
     * @return std::map<char, std::vector<float>>
     */
    std::map<char, std::vector<float>> get_armBindings();

private:
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings{
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
        {'K', {0, 0, 0, 0}}};

    // Map for speed keys
    std::map<char, std::vector<float>> speedBindings{
        {'q', {1.1, 1.1}},
        {'z', {0.9, 0.9}},
        {'w', {1.1, 1}},
        {'x', {0.9, 1}},
        {'e', {1, 1.1}},
        {'c', {1, 0.9}}};
    // Create map armBindings
    std::map<char, std::vector<float>> armBindings{
        {'1', {0.0, 0}},
        {'2', {-0.1, 0}}};
};
#endif // INCLUDE_DVD_ROBOT_AVOID_WALLS_HPP_