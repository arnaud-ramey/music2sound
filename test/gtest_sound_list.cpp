/*!
  \file        gtest_sound_list.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/17

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

Some tests for EttsSkill and EttsApi.
 */
#include "music2sound/sox_generator.h"
#include <math.h>
// Bring in gtest
#include <gtest/gtest.h>

std::string PATH = ros::package::getPath("music2sound");

TEST(TestSuite, score_from_string) {
  Score score;
  std::string s = "";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 0);

  s = ",";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 0);

  s = ",,,,";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 0);

  s = "BPM=0";
  ASSERT_FALSE(score.from_string(s));

  s = "BPM=800";
  ASSERT_FALSE(score.from_string(s));

  s = "BPM=80";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 1);
  ScoreElem e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_BPM && e.bpm == 80);

  s = "BPM=80, BPM=200";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 2);
  e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_BPM && e.bpm == 80);
  e = score.notes[1];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_BPM && e.bpm == 200);

  s = "volume=80, volume=200";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 2);
  e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_VOLUME && e.volume == 80);
  e = score.notes[1];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_VOLUME && e.volume == 200);

  s = "BPM=80, 1-A1";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 2);
  e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_BPM && e.bpm == 80);
  e = score.notes[1];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_NOTE
              && e.note_duration == 1 && e.note_name == "A1");

  s = "BPM=80, 1-A1";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 2);
  e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_BPM && e.bpm == 80);
  e = score.notes[1];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_NOTE
              && e.note_duration == 1 && e.note_name == "A1");

  s = "1/2-A1,1/4-B2,2-C3";
  ASSERT_TRUE(score.from_string(s));
  ASSERT_TRUE(score.notes.size() == 3);
  e = score.notes[0];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_NOTE
              && e.note_duration == .5 && e.note_name == "A1");
  e = score.notes[1];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_NOTE
              && e.note_duration == .25 && e.note_name == "B2");
  e = score.notes[2];
  ASSERT_TRUE(e.type == ScoreElem::TYPE_NOTE
              && e.note_duration == 2 && e.note_name == "C3");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, soundlist_from_score) {
  Score score;
  SoundList sl;
  SoundList::TimedNote tnote;
  ASSERT_TRUE(score.from_string(""));
  ASSERT_TRUE(sl.from_score(score));
  ASSERT_TRUE(sl.duration == 0);
  ASSERT_TRUE(sl.tnotes.size() == 0);

  ASSERT_TRUE(score.from_string("BPM=80"));
  ASSERT_TRUE(sl.from_score(score));
  ASSERT_TRUE(sl.duration == 0);
  ASSERT_TRUE(sl.tnotes.size() == 0);

  ASSERT_TRUE(score.from_string("BPM=80, 1-A1"));
  ASSERT_TRUE(sl.from_score(score));
  ASSERT_TRUE(fabs(sl.duration - 1./80) < 1E-2);
  ASSERT_TRUE(sl.tnotes.size() == 1);
  tnote = sl.tnotes[0];
  ASSERT_TRUE(tnote.time == 0 && tnote.volume == 100 && tnote.note_name == "A1");

  ASSERT_TRUE(score.from_string("BPM=10, volume=95,1/2-A1,1/4-B2, volume=90,2-C3"));
  ASSERT_TRUE(sl.from_score(score));
  ASSERT_TRUE(sl.tnotes.size() == 3);
  ASSERT_TRUE(fabs(sl.duration - (.5+.25+2)/10) < 1E-2);
  tnote = sl.tnotes[0];
  ASSERT_TRUE(tnote.time == 0 && tnote.volume == 95 && tnote.note_name == "A1");
  tnote = sl.tnotes[1];
  ASSERT_TRUE(fabs(tnote.time-.05)<1E-2
              && tnote.volume == 95 && tnote.note_name == "B2");
  tnote = sl.tnotes[2];
  ASSERT_TRUE(fabs(tnote.time-.075)<1E-2
              && tnote.volume == 90 && tnote.note_name == "C3");

  ASSERT_TRUE(score.from_string("BPM=10, 1/2-A1,1/2-{}, B2, {}, C3"));
  ASSERT_TRUE(sl.from_score(score));
  ASSERT_TRUE(sl.tnotes.size() == 5);
  ASSERT_TRUE(fabs(sl.duration - .4) < 1E-2);
  tnote = sl.tnotes[0];
  ASSERT_TRUE(tnote.time == 0 && tnote.volume == 100 && tnote.note_name == "A1");
  tnote = sl.tnotes[2];
  ASSERT_TRUE(fabs(tnote.time-.1)<1E-2
              && tnote.volume == 100 && tnote.note_name == "B2");
  tnote = sl.tnotes[4];
  ASSERT_TRUE(fabs(tnote.time-.3)<1E-2
              && tnote.volume == 100 && tnote.note_name == "C3");
}

////////////////////////////////////////////////////////////////////////////////
void test_play(const std::string & s) {
  SoxGenerator prim;
  prim.set_path_prefix(PATH + "/data/piano/Piano.ff.");
  prim.set_path_suffix(".wav");
  ASSERT_TRUE(prim.generate(s));
  sleep(1);
}

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
