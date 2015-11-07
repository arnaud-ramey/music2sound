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
#ifndef SOX_GENERATOR_H
#define SOX_GENERATOR_H

#include "music2sound/sound_list.h"
#include "music2sound/cached_files_map.h"
#include <ros/package.h>

#define CACHE_WAV_CSV_FILE (ros::package::getPath("music2sound") + "/data/SoxGenerator_wav_cache/index.csv")
#define WAV_BUFFER ("/tmp/SoxGenerator.wav")

class SoxGenerator {
public:
  SoxGenerator() : _wav_map(CACHE_WAV_CSV_FILE) {}

  bool generate(const std::string & score) {
    std::string score_str = score;
    // read file if it is a file
    if (score.find(".score") != std::string::npos
        && !utils::retrieve_file(score, score_str)) {
      printf("Could not read score file'%s'!\n", score.c_str());
      return false;
    }

    std::ostringstream instr;
    CachedFilesMap::Key key = score_str;
    std::string curr_wav_filename = WAV_BUFFER;
    if (_wav_map.has_cached_file(key)
        && _wav_map.get_cached_file(key, curr_wav_filename)) {
      printf("SoxGenerator: sentence '%s' was already in cache:'%s'\n",
             score_str.c_str(), curr_wav_filename.c_str());
    }
    else {
      int BPM = SoundList::DEFAULT_BPM;
      if (!_score.from_string(score_str) || !_sound_list.from_score(_score, BPM))
        return false;
      if (_sound_list.tnotes.empty())
        return true;
      unsigned int nnotes = _sound_list.tnotes.size(), nsilences = 0;
      printf("SoxGenerator: synthetizing a sentence of %i notes...\n", nnotes);
      // create an empty sound with SoX
      // http://activearchives.org/wiki/Padding_an_audio_file_with_silence_using_sox
      double duration = _sound_list.duration;
      instr << "sox -n -r 44100 -b 16 -c 2 " << WAV_BUFFER << " trim 0 " << duration;
      if (utils::exec_system(instr.str()))
        return false;
      // copy paste the notes
      // http://sox.sourceforge.net/Docs/FAQ
      //  sox -m f1.wav "|sox f2.wav -p pad 4" "|sox f3.wav -p pad 8" out.wav
      // sox -m $OUT "|sox $FILE -p pad 1" "|sox $FILE -p pad .5"  "|sox $FILE -p pad 1.5" --norm $OUT
      instr.str("");
      instr << "sox -m " << WAV_BUFFER;
      for (int i = 0; i < nnotes; ++i) {
        SoundList::TimedNote* curr_note = &(_sound_list.tnotes[i]);
        if (curr_note->note_name == "{}") { // silence
          ++nsilences;
          continue;
        }
        instr << " \"| sox " << _path_prefix << curr_note->note_name << _path_suffix
              << " -p pad "<< curr_note->time << "\" ";
      } // end for i
      instr << " --norm=-3 " << WAV_BUFFER;
      instr << " 2> /dev/null";
      // do not play if only silences
      if (nsilences == nnotes)
        return true;
      //printf("creating sound '%s' \n", instr.str().c_str());
      if (utils::exec_system(instr.str()))
        return false;
      curr_wav_filename = WAV_BUFFER;
      _wav_map.add_cached_file(key, WAV_BUFFER);
    } // end if not cached
    // play resulting file
    instr.str("");
    instr << "play -q " << curr_wav_filename;
    if (utils::exec_system(instr.str()))
      return false;
    // play sound
    return true;
  } // end generate()

  inline void set_path_prefix(const std::string & path_prefix) {
    _path_prefix = path_prefix;
  }
  inline void set_path_suffix(const std::string & path_suffix) {
    _path_suffix = path_suffix;
  }

  std::string _path_prefix, _path_suffix;
  int _volume;
  Score _score;
  SoundList _sound_list;
  CachedFilesMap _wav_map;
}; // end class SoxGenerator

#endif // SOX_GENERATOR_H

