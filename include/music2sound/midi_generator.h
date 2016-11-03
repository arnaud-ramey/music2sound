/*!
  \file        midi_generator.h
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
#ifndef MIDI_GENERATOR_H
#define MIDI_GENERATOR_H

#include "music2sound/sound_list.h"
#include "music2sound/midifile.h"
#include "music2sound/cached_files_map.h"
#include <ros/package.h>

#define NOTE2MIDI_NOTE_NAME_FILE (ros::package::getPath("music2sound") + "/data/note2midi_note_name.csv")
#define CACHE_MIDI_CSV_FILE (ros::package::getPath("music2sound") + "/data/MidiGenerator_midi_cache/index.csv")
#define MIDI_BUFFER ("/tmp/MidiGenerator.MIDI")

class MidiGenerator {
public:
  //! convert seconds into Midi ticks duration
  static const double SEC2TICKS;

  MidiGenerator() : _midi_map(CACHE_MIDI_CSV_FILE) {
    // parse NOTE2MIDI_NOTE_NAME_FILE
    std::vector<std::string> lines, words;
    if (!utils::retrieve_file_split(NOTE2MIDI_NOTE_NAME_FILE, lines)) {
      printf("Could not parse '%s'\n", NOTE2MIDI_NOTE_NAME_FILE.c_str());
      return;
    }
    unsigned int nlines = lines.size();
    for (unsigned int i = 0; i < nlines; ++i) {
      if (lines[i].size() && lines[i][0] == '#') // comment
        continue; // skip line
      utils::StringSplit(lines[i], ",", &words);
      if (words.size() != 2) {
        printf("Could not parse line '%s'\n", lines[i].c_str());
        return;
      }
      std::string note = words.front();
      int name = utils::cast_from_string<int>(words.back());
      _note2midi_note_name.insert(std::make_pair(note, name));
    } // end for i
    printf("Parsed %li notes\n", _note2midi_note_name.size());
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \return the number of genereted notes, 0 if empty or in cache, or -1 if failure
  int generate(const std::string & score, bool skip_cache = false) {
    std::string score_str = score;
    // read file if it is a file
    if (score.find(".score") != std::string::npos
        && !utils::retrieve_file(score, score_str)) {
      printf("Could not read score file'%s'!\n", score.c_str());
      return -1;
    }

    CachedFilesMap::Key key = score_str;
    std::string curr_MIDI_filename = MIDI_BUFFER;
    unsigned int nnon_silent = 0;
    if (!skip_cache
        && _midi_map.has_cached_file(key)
        && _midi_map.get_cached_file(key, curr_MIDI_filename)) {
      printf("MidiGenerator: sentence '%s' was already in cache:'%s'\n",
             score_str.c_str(), curr_MIDI_filename.c_str());
    }
    else {
      int BPM = SoundList::DEFAULT_BPM;
      if (!_score.from_string(score_str) || !_sound_list.from_score(_score, BPM))
        return -1;
      if (_sound_list.tnotes.empty())
        return 0;
      unsigned int nnotes = _sound_list.tnotes.size(), nsilences = 0;
      printf("MidiGenerator: synthetizing a sentence of %i notes...\n", nnotes);
      _sequence.clear();
      // copy paste the notes
      for (unsigned int i = 0; i < nnotes; ++i) {
        SoundList::TimedNote* curr_note = &(_sound_list.tnotes[i]);
        if (!_note2midi_note_name.count(curr_note->note_name)) {
          printf("MidiGenerator: not found note '%s'\n", curr_note->note_name.c_str());
          continue;
        }
        int midi_note_name = _note2midi_note_name[curr_note->note_name];
        if (midi_note_name == -1)
          ++nsilences;
        _sequence.push_back(midi_note_name);
        _sequence.push_back(curr_note->duration * SEC2TICKS);
      } // end for i
      // display
      std::ostringstream vec2str;
      for (unsigned int i = 0; i < _sequence.size(); ++i)
        vec2str << _sequence[i] <<", ";
      printf("vec2str:'%s'\n", vec2str.str().c_str());

      nnon_silent = (_sequence.size()/2 - nsilences);
      _file.noteSequenceFixedVelocity(_sequence, 127);
      _file.writeToFile (MIDI_BUFFER);
      curr_MIDI_filename = MIDI_BUFFER;
      _midi_map.add_cached_file(key, MIDI_BUFFER);
    } // end if not cached
    // play resulting file
    std::ostringstream instr;
    instr << "timidity " << curr_MIDI_filename;
    if(utils::exec_system(instr.str()))
      return -1;
    return nnon_silent;
  } // end generate()

  int _volume;
  Score _score;
  SoundList _sound_list;
  CachedFilesMap _midi_map;
  MidiFile _file;
  std::vector<int> _sequence;
  std::map<std::string, int> _note2midi_note_name;
}; // end class MidiGenerator

// MidiFile: 1 million usec = 1E-3 sec per crotchet, 1 crotchet = 16 ticks
// 1 crotchet at 60 BPM = 1/4 seconds = 16 ticks
const double MidiGenerator::SEC2TICKS = 16;

#endif // MIDI_GENERATOR_H

