/*!
  \file        midifile.h
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

C++ translation of http://kevinboone.net/javamidi.html
 */
#ifndef MIDIFILE_H
#define MIDIFILE_H
#include <vector>
#include <string>
#include <fstream>
typedef unsigned char byte;

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v) {
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    os << *ii;
  return os;
}

class MidiFile {
public:
  // Note lengths
  //  We are working with 32 ticks to the crotchet. So
  //  all the other note lengths can be derived from this
  //  basic figure. Note that the longest note we can
  //  represent with this code is one tick short of a
  //  two semibreves (i.e., 8 crotchets)
  static const int SEMIQUAVER;
  static const int QUAVER;
  static const int CROTCHET;
  static const int MINIM;
  static const int SEMIBREVE;

  /** Construct a new MidiFile with an empty playback event list */
  MidiFile() {
    static const int arr1[] = {
      0x4d, 0x54, 0x68, 0x64, 0x00, 0x00, 0x00, 0x06,
      0x00, 0x00, // single-track format
      0x00, 0x01, // one track
      0x00, 0x10, // 16 ticks per quarter
      0x4d, 0x54, 0x72, 0x6B
    };
    header = std::vector<int> (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );

    static const int arr2[] = {
      0x01, 0xFF, 0x2F, 0x00
    };
    footer = std::vector<int> (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );

    // A MIDI event to set the tempo
    static const int arr3[] = {
      0x00, 0xFF, 0x51, 0x03,
      0x0F, 0x42, 0x40 // Default 1 million usec per crotchet
    };
    tempoEvent = std::vector<int> (arr3, arr3 + sizeof(arr3) / sizeof(arr3[0]) );

    // A MIDI event to set the key signature. This is irrelevent to
    //  playback, but necessary for editing applications
    static const int arr4[] = {
      0x00, 0xFF, 0x59, 0x02,
      0x00, // C
      0x00  // major
    };
    keySigEvent = std::vector<int> (arr4, arr4 + sizeof(arr4) / sizeof(arr4[0]) );

    // A MIDI event to set the time signature. This is irrelevent to
    //  playback, but necessary for editing applications
    static const int arr5[] = {
      0x00, 0xFF, 0x58, 0x04,
      0x04, // numerator
      0x02, // denominator (2==4, because it's a power of 2)
      0x30, // ticks per click (not used)
      0x08  // 32nd notes per crotchet
    };
    timeSigEvent = std::vector<int> (arr5, arr5 + sizeof(arr5) / sizeof(arr5[0]) );
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  /** Write the stored MIDI events to a file */
  void writeToFile (const std::string & filename) {
    std::ofstream fos (filename.c_str());
    fos << intArrayToByteArray (header);

    // Calculate the amount of track data
    // _Do_ include the footer but _do not_ include the
    // track header
    int size = tempoEvent.size() + keySigEvent.size() + timeSigEvent.size()
               + footer.size();
    for (unsigned int i = 0; i < playEvents.size(); i++)
      size += playEvents.at(i).size();

    // Write out the track data size in big-endian format
    // Note that this math is only valid for up to 64k of data
    //  (but that's a lot of notes)
    int high = size / 256;
    int low = size - (high * 256);
    fos << (byte) 0;
    fos << (byte) 0;
    fos << (byte) high;
    fos << (byte) low;

    // Write the standard metadata — tempo, etc
    // At present, tempo is stuck at crotchet=60
    fos << intArrayToByteArray (tempoEvent);
    fos << intArrayToByteArray (keySigEvent);
    fos << intArrayToByteArray (timeSigEvent);


    // Write out the note, etc., events
    for (unsigned int i = 0; i < playEvents.size(); i++)
      fos << intArrayToByteArray (playEvents.at(i));

    // Write the footer and close
    fos << intArrayToByteArray (footer);
    fos.close();
  } // end writeToFile();

  //////////////////////////////////////////////////////////////////////////////

  /** Store a note-on event */
  void noteOn (int delta, int note, int velocity) {
    std::vector<int> data(4);
    data[0] = delta;
    data[1] = 0x90;
    data[2] = note;
    data[3] = velocity;
    playEvents.push_back(data);
  }

  //////////////////////////////////////////////////////////////////////////////

  /** Store a note-off event */
  void noteOff (int delta, int note) {
    std::vector<int> data(4);
    data[0] = delta;
    data[1] = 0x80;
    data[2] = note;
    data[3] = 0;
    playEvents.push_back (data);
  }

  //////////////////////////////////////////////////////////////////////////////

  /** Store a program-change event at current position */
  void progChange (int prog) {
    std::vector<int> data(3);
    data[0] = 0;
    data[1] = 0xC0;
    data[2] = prog;
    playEvents.push_back (data);
  }

  //////////////////////////////////////////////////////////////////////////////

  /** Store a note-on event followed by a note-off event a note length
      later. There is no delta value — the note is assumed to
      follow the previous one with no gap. */
  void noteOnOffNow (int duration, int note, int velocity) {
    noteOn (0, note, velocity);
    noteOff (duration, note);
  }

  //////////////////////////////////////////////////////////////////////////////

  void noteSequenceFixedVelocity (const std::vector<int> & sequence, int velocity) {
    bool lastWasRest = false;
    int restDelta = 0;
    for (unsigned int i = 0; i < sequence.size(); i += 2) {
      int note = sequence[i];
      int duration = sequence[i + 1];
      if (note < 0) {
        // This is a rest
        restDelta += duration;
        lastWasRest = true;
      }
      else {
        // A note, not a rest
        if (lastWasRest) {
          noteOn (restDelta, note, velocity);
          noteOff (duration, note);
        }
        else {
          noteOn (0, note, velocity);
          noteOff (duration, note);
        }
        restDelta = 0;
        lastWasRest = false;
      } // end (note > 0)
    } // end for i
  } // end noteSequenceFixedVelocity()

  //////////////////////////////////////////////////////////////////////////////
protected:

  /** Convert an array of integers which are assumed to contain
      unsigned bytes into an array of bytes */
  static std::vector<byte>  intArrayToByteArray (const std::vector<int> & ints) {
    int l = ints.size();
    std::vector<byte>  out(ints.size());
    for (int i = 0; i < l; i++) {
      out[i] = (byte) ints[i];
    }
    return out;
  }

  //////////////////////////////////////////////////////////////////////////////

  // Standard MIDI file header, for one-track file
  // 4D, 54... are just magic numbers to identify the
  //  headers
  // Note that because we're only writing one track, we
  //  can for simplicity combine the file and track headers
  std::vector<int> header;

  // Standard footer
  std::vector<int> footer;

  // A MIDI event to set the tempo
  std::vector<int> tempoEvent;

  // A MIDI event to set the key signature. This is irrelent to
  //  playback, but necessary for editing applications
  std::vector<int> keySigEvent;

  // A MIDI event to set the time signature. This is irrelent to
  //  playback, but necessary for editing applications
  std::vector<int> timeSigEvent;

  // The collection of events to play, in time order
  std::vector<std::vector<int> > playEvents;
}; // end class MidiFile

// https://stackoverflow.com/questions/5391973/undefined-reference-to-static-const-int
const int MidiFile::SEMIQUAVER = 4;
const int MidiFile::QUAVER = 8;
const int MidiFile::CROTCHET = 16; // (British) = quarter (US)
const int MidiFile::MINIM = 32;
const int MidiFile::SEMIBREVE = 64; // (British) = whole note (US)

#endif // MIDIFILE_H
