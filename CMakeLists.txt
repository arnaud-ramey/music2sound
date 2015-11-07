### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(music2sound)
### 3 - Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS
  roscpp  roslib)
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
### 5 - Invoke message/service/action generation (generate_messages())
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  roslib
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories( include ${catkin_INCLUDE_DIRS})
add_executable(test_sox_generator src/test_sox_generator.cpp)
target_link_libraries(test_sox_generator  ${catkin_LIBRARIES})
### 8 - Tests to build (catkin_add_gtest())
catkin_add_gtest(gtest_sound_list test/gtest_sound_list.cpp
            include/music2sound/cached_files_map.h
            include/music2sound/sound_list.h
            include/music2sound/sox_generator.h
            include/music2sound/utils.h)
target_link_libraries(gtest_sound_list  ${catkin_LIBRARIES})
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
