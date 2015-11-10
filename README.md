music2sound
===========

music2sound is a sound generator using music scores (also called music sheets).

Scores
------
A simple music writing language has been developed,
for example: "BPM=85,A6,Ab2,C5,1/4-C3"

Generators
----------
Generators are pieces of code capable of generating a sound from
music written in this language.
To date, three generators are offered:

  1. **BeepGenerator**: based on the "beep" utility.
    Uses the sequencing features of "beep".

    beep is available in the Ubuntu repos.
    More info:
    http://linux.die.net/man/1/beep

  2. **MidiGenerator**: a MIDI filewriter, coupled with the "TiMidity++" synthethizer.
    The MIDI file writing is based on the Java MIDI file writing sample
    of Kevin Boone :
    http://kevinboone.net/javamidi.html

    TiMidity++ is available in the Ubuntu repos.
    More info:
    http://timidity.sourceforge.net/

  3. **SoxGeneratpr**: based on WAV sounds of piano keys and SoX audio copy/pasting.
    Piano samples from http://theremin.music.uiowa.edu/MIS.html

    SoX is available in the Ubuntu repos.
    More info:
    http://sox.sourceforge.net/Docs/FAQ

In the future, more generators might be implemented.

Command-line tests
------------------
For each generator, a simple command-line test is supplied.

For example, to play a single note with MidiGenerator:

$ rosrun music2sound test_midi_generator "A4"

To play a score stored in a file :

$ rosrun music2sound test_midi_generator "`rospack find music2sound`/data/music_scores/tetris.score"


ROS wrappers
------------
A simple ROS wrapper has been implemented for each of the generators.
It is called "ros_*_generator", for instance "ros_sox_generator".
It subscribes to a "std_msgs::String" message and generates the corresponding sound.

# Tests

You need three terminals.
For example, with MidiGenerator:

T1: $ roscore

T2: $ rosrun music2sound ros_midi_generator

To play a single note :

T3: $ rostopic pub /ros_midi_generator/msg  std_msgs/String  "data: 'A6'"

To play a score stored in a file :

T3: $ rostopic pub /ros_midi_generator/msg  std_msgs/String  "data: '`rospack find music2sound`/data/music_scores/tetris.score'"
