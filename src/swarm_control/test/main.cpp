/**
 * @file main.cpp
 * @author Vikram Setty (vikrams@umd.edu)
 * @brief The file that sets uo the Gtest suite for the entire library
 * @version 0.1
 * @date 2023-12-09
 * 
 * @copyright Copyright (c) 2023 Vikram Setty, Vinay Lanka

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 * 
 */
#include <gtest/gtest.h>

/**
 * @brief A function called in case a function is not used
 * 
 * @return * void 
 */
void function_not_used() {
  int notUsed = 3;
}

/**
 * @brief Setting up the Gtest suite
 * 
 * @param argc 
 * @param argv 
 * @return * int 
 */
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
