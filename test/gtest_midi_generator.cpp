/*!
  \file        gtest_midi_generator.cpp
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

Some tests for music generation with MidiGenerator.
 */
#include "music2sound/midi_generator.h"
// Bring in gtest
#include <gtest/gtest.h>

std::string PATH = ros::package::getPath("music2sound");

//! Test 1 — play a C major chord
TEST(TestSuite, midifile_test1) {
  MidiFile mf;
  // Turn on all three notes at start-of-track (delta=0)
  mf.noteOn (0, 60, 127);
  mf.noteOn (0, 64, 127);
  mf.noteOn (0, 67, 127);

  // Turn off all three notes after one minim.
  // NOTE delta value is cumulative — only _one_ of
  //  these note-offs has a non-zero delta. The second and
  //  third events are relative to the first
  mf.noteOff (MidiFile::MINIM, 60);
  mf.noteOff (0, 64);
  mf.noteOff (0, 67);
  mf.writeToFile ("test1.mid");
}

//! Test 2 — play a scale using noteOnOffNow
TEST(TestSuite, midifile_test2) {
  MidiFile mf;
  //  We don't need any delta values here, so long as one
  //  note comes straight after the previous one
  mf.noteOnOffNow (MidiFile::QUAVER, 60, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 62, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 64, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 65, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 67, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 69, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 71, 127);
  mf.noteOnOffNow (MidiFile::QUAVER, 72, 127);
  mf.writeToFile ("test1.mid");
}

//! Test 3 — play a short tune using noteSequenceFixedVelocity
TEST(TestSuite, midifile_test3) {
  MidiFile mf;
  //  Note the rest inserted with a note value of -1
  static const int arr1[] = {
    60, MidiFile::QUAVER + MidiFile::SEMIQUAVER,
    65, MidiFile::SEMIQUAVER,
    70, MidiFile::CROTCHET + MidiFile::QUAVER,
    69, MidiFile::QUAVER,
    65, MidiFile::QUAVER / 3,
    62, MidiFile::QUAVER / 3,
    67, MidiFile::QUAVER / 3,
    72, MidiFile::MINIM + MidiFile::QUAVER,
    -1, MidiFile::SEMIQUAVER,
    72, MidiFile::SEMIQUAVER,
    76, MidiFile::MINIM,
  };
  std::vector<int> sequence (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );

  // What the heck — use a different instrument for a change
  //mf.progChange (10);
  mf.noteSequenceFixedVelocity (sequence, 127);
  mf.writeToFile ("test1.mid");
}

////////////////////////////////////////////////////////////////////////////////

void test_play(const std::string & s) {
  MidiGenerator prim;
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
