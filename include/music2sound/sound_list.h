/*!
  \file        sound_list.h
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

A TTS engine based on reading scores (music sheets)
and playing them with a music instrument.
 */
#ifndef SOUND_LIST_H
#define SOUND_LIST_H

#include <string>
#include <vector>
#include <map>
#include "music2sound/utils.h"

/*! Notes with duration: \example
 * crotchet silence:  1-{}
 * crotchet:          1-C3
 * quaver:            1/2-C3
 * semiquaver:        1/4-C3
 * triplet:           1/3-C3
 */
class ScoreElem {
public:
  enum Type {
    TYPE_NOTE = 1,
    TYPE_BPM = 2,
    TYPE_VOLUME = 3
  };

  ScoreElem() : type(TYPE_NOTE), note_duration(1), bpm(1), volume(100) {}

  bool from_string(const std::string & in) {
    if (in.substr(0, 3) == "BPM" || in.substr(0, 3) == "bpm") { // BPM:
      type = TYPE_BPM;
      bpm = utils::cast_from_string<int>(in.substr(4));
      if (bpm < 1 || bpm > 300) {
        printf("ScoreElem::from_string(): incorrect BPM '%s'\n", in.c_str());
        return false;
      }
      return true;
    }
    if (in.substr(0, 6) == "volume" || in.substr(0, 6) == "VOLUME") { // VOLUME:
      type = TYPE_VOLUME;
      volume = utils::cast_from_string<int>(in.substr(7));
      if (volume < 1 || volume > 300) {
        printf("ScoreElem::from_string(): incorrect VOLUME '%s'\n", in.c_str());
        return false;
      }
      return true;
    }
    // parse note
    type = TYPE_NOTE;
    note_duration = 1;
    note_name = in;
    // parse duration
    size_t hyphen_pos = in.find('-');
    if (hyphen_pos != std::string::npos) {
      std::string duration_str = in.substr(0, hyphen_pos);
      if (duration_str == "1")
        note_duration = 1;
      else if (duration_str == "1/2")
        note_duration = .5;
      else if (duration_str == "1/4")
        note_duration = .25;
      else if (duration_str == "3/4")
        note_duration = .75;
      else if (duration_str == "1/8")
        note_duration = .125;
      else if (duration_str == "1/3")
        note_duration = 1./3;
      else {
        note_duration = utils::cast_from_string<double>(duration_str);
        if (note_duration < .1 || note_duration > 10) {
          printf("ScoreElem::from_string(): incorrect duration '%s'\n", duration_str.c_str());
          return false;
        }
      }
      note_name = in.substr(hyphen_pos+1);
    }
    return true;
  } // end ScoreElem::from_string()

  Type type;
  std::string note_name;
  double note_duration;
  int bpm;
  int volume; // 0 - 100
}; // end class ScoreElem

////////////////////////////////////////////////////////////////////////////////

//! \example BPM=85,A6,Ab2,C5,1/4-C3
class Score {
public:
  bool from_string(const std::string & in) {
    notes.clear();
    if (in.empty()) // empty score
      return true;
    // split properties and notes
    std::vector<std::string> notes_str;
    utils::StringSplit(in, ",", &notes_str);
    if (notes_str.size() == 0) // empty score
      return true;
    ScoreElem elem;
    for (unsigned int note_idx = 0; note_idx < notes_str.size(); ++note_idx) {
      std::string note_str = notes_str[note_idx];
      utils::remove_beginning_spaces(note_str);
      utils::remove_trailing_spaces(note_str);
      if (note_str.empty() || note_str.substr(0, 1) == "#") // comment
        continue;
      if (elem.from_string(note_str)) {
        notes.push_back(elem);
        continue;
      }
      printf("Score::from_string(): cant make sense of element '%s'\n", note_str.c_str());
      return false;
    } // end loop note_idx
    return true;
  } // end Score::from_string()

  std::vector<ScoreElem> notes;
}; // end class Score

////////////////////////////////////////////////////////////////////////////////

class SoundList {
public:
  static const int DEFAULT_BPM = 5;
  typedef std::string NoteName;
  typedef double Time;
  struct TimedNote {
    NoteName note_name; // for ex. "A6"
    Time time; // seconds
    double duration; // seconds
    int volume; // 0 .. 100
  };
  bool from_score(const Score & s, int BPM = DEFAULT_BPM) {
    unsigned int nnotes = s.notes.size();
    tnotes.clear();
    tnotes.reserve(nnotes);
    duration = 0;
    double curr_time = 0;
    double curr_bpm_inv = 1. / BPM;
    int curr_volume = 100;
    for (unsigned int note_idx = 0; note_idx < nnotes; ++note_idx) {
      const ScoreElem* e = &(s.notes[note_idx]);
      if (e->type == ScoreElem::TYPE_VOLUME) {
        curr_volume = e->volume;
      }
      else if (e->type == ScoreElem::TYPE_BPM) {
        if (e->bpm == 0) {
          printf("Score::from_string(): BPM of zero!\n");
          return false;
        }
        curr_bpm_inv = 1. / e->bpm;
      }
      else if (e->type == ScoreElem::TYPE_NOTE) {
        //if (e->note_name != "{}")  { // not a silence
          TimedNote tnote;
          tnote.time = curr_time;
          tnote.note_name = e->note_name;
          tnote.volume = curr_volume;
          tnote.duration = e->note_duration * curr_bpm_inv;
          tnotes.push_back(tnote);
        //}
        curr_time += e->note_duration * curr_bpm_inv;
      }
    } // end loop note_idx
    duration = curr_time;
    return true;
  } // end SoundList::from_score()

  std::vector<TimedNote> tnotes;
  double duration; // time of last note + duration of last note
};

#endif // SOUND_LIST_H
