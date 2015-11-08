/*!
  \file        gtest_beep_generator.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/11/7

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Some tests for music generation with BeepGenerator.
 */
#include "music2sound/beep_generator.h"
// Bring in gtest
#include <gtest/gtest.h>

std::string PATH = ros::package::getPath("music2sound");

void test_play(const std::string & s) {
  BeepGenerator prim;
  ASSERT_TRUE(prim.generate(s));
  sleep(1);
}

TEST(TestSuite, play_empty1) { test_play("{}"); }
TEST(TestSuite, play_empty2) { test_play("BPM=10"); }
TEST(TestSuite, play_wrong1) { test_play("A12"); }
TEST(TestSuite, play01) { test_play("A6"); }
TEST(TestSuite, play02) { test_play("A4,{},A4"); }
TEST(TestSuite, play03) { test_play("BPM=2, C5,D5,E5,F5,G5,A5,B5"); }
TEST(TestSuite, play04) { test_play("E6, D6, C6, D6, E6"); }
TEST(TestSuite, play05) { test_play("BPM:10, E6, D6, C6, D6, E6"); }
TEST(TestSuite, play_file1) { test_play(PATH + "/data/music_scores/happy_birthday.score"); }
TEST(TestSuite, play_file2) { test_play(PATH + "/data/music_scores/happy_birthday2.score"); }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
