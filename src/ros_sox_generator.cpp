/*!
  \file        ros_sox_generator.cpp
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

A thin ROS wrapper for SoxGenerator.

To test:
rostopic pub /ros_sox_generator/msg std_msgs/String "data: 'A6'"
 */
#include "music2sound/sox_generator.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

std::string PATH = ros::package::getPath("music2sound");
SoxGenerator prim;

void msg_cb(const std_msgs::String::ConstPtr & msg) {
  prim.generate(msg->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_sox_generator");
  prim.set_path_prefix(PATH + "/data/piano/Piano.ff.");
  prim.set_path_suffix(".wav");
  ros::NodeHandle nh_private("~");
  ros::Subscriber sub = nh_private.subscribe("msg", 1, msg_cb);
  ros::spin();
}
