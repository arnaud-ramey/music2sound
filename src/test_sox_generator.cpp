/*!
  \file        test_sox_generator.cpp
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
#include "music2sound/sox_generator.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("Synopsis: %s <score>\n", argv[0]);
    return -1;
  }
  std::string score = argv[1];
  SoxGenerator prim;
  prim.set_path_prefix(ros::package::getPath("music2sound") + "/data/piano/Piano.ff.");
  prim.set_path_suffix(".wav");
  return (prim.generate(score) ? 0 : -1);
}

