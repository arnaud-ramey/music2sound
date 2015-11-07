/*!
  \file
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

\todo Description of the file
 */

#ifndef BEEP_GENERATOR_H
#define BEEP_GENERATOR_H

#include "music2sound/sound_list.h"
#include <ros/package.h>

#define NOTE2FREQUENCY_FILE (ros::package::getPath("music2sound") + "/data/note2frequency.csv")

class BeepGenerator {
public:
  BeepGenerator() {
    // parse NOTE2FREQUENCY_FILE
    std::vector<std::string> lines, words;
    if (!utils::retrieve_file_split(NOTE2FREQUENCY_FILE, lines)) {
      printf("Could not parse '%s'\n", NOTE2FREQUENCY_FILE.c_str());
      return;
    }
    unsigned int nlines = lines.size();
    for (int i = 0; i < nlines; ++i) {
      if (lines[i].size() && lines[i][0] == '#') // comment
        continue; // skip line
      utils::StringSplit(lines[i], ",", &words);
      if (words.size() != 2) {
        printf("Could not parse line '%s'\n", lines[i].c_str());
        return;
      }
      std::string note = words.front();
      double freq = utils::cast_from_string<double>(words.back());
      _note2frequency.insert(std::make_pair(note, freq));
    } // end for i
    printf("Parsed %i notes\n", _note2frequency.size());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  bool generate(const std::string & score) {
    std::string score_str = score;
    // read file if it is a file
    if (score.find(".score") != std::string::npos
        && !utils::retrieve_file(score, score_str))
      return false;

    std::ostringstream instr;
    int BPM = SoundList::DEFAULT_BPM;
    if (!_score.from_string(score_str) || !_sound_list.from_score(_score, BPM))
      return false;
    if (_sound_list.tnotes.empty())
      return true;
    unsigned int nnotes = _sound_list.tnotes.size(), nsilences = 0;
    // beep -f 1000 -r 2 -n -r 5 -l 10 --new
    // will produce first two 1000Hz beeps, then 5 beeps at the default tone,
    // but only 10ms long each, followed by
    // a third beep using all the default settings (since none are specified)
    instr << "beep";
    for (int i = 0; i < nnotes; ++i) {
      SoundList::TimedNote* curr_note = &(_sound_list.tnotes[i]);
      if (curr_note->note_name == "{}") { // silence
        if (i - nsilences > 0) // only add -D if there was a note before
          instr << " -D " << curr_note->duration * 1000;
        ++nsilences;
        continue;
      }
      if (i - nsilences > 0)
        instr << "  --new";
      double frequency_hz = _note2frequency[curr_note->note_name];
      double duration_ms = curr_note->duration * 1000;
      if (curr_note->note_name == "{}") // silence
        continue;
      instr << " -f " << frequency_hz
            << " -l " << duration_ms;
    } // end for i
    // do not play if only silences
    if (nsilences == nnotes)
      return true;
    printf("instr:'%s'\n", instr.str().c_str());
    return utils::exec_system(instr.str());
  } // end generate()

  int _volume;
  Score _score;
  SoundList _sound_list;
  std::map<std::string, float> _note2frequency;
}; // end class BeepGenerator

#endif // BEEP_GENERATOR_H

