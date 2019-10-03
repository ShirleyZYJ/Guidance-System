cmake_minimum_required(VERSION 2.8.3)
project(visguide)


find_package(catkin REQUIRED COMPONENTS
  #rtabmap
  rtabmap_ros
  zed_wrapper
  roslaunch
  cv_bridge
  nav_msgs
)

find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

 generate_messages(
 # DEPENDENCIES
 # 	nav_msgs
 #  std_msgs  # Or other packages containing msgs
 
 )

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visguide
#  CATKIN_DEPENDS rtabmap rtabmap_ros zed_wrapper
#  DEPENDS system_lib

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
roslaunch_add_file_check(launch)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/visguide.cpp
# )

 
 add_executable(${PROJECT_NAME}_node src/Sound_comm.cpp)
 add_executable(${PROJECT_NAME}_node2 src/plan_to_text.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(${PROJECT_NAME}_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 #set_target_properties(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})#PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS} )
   
  target_link_libraries(${PROJECT_NAME}_node2
   ${catkin_LIBRARIES} ${OpenCV_LIBS} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_visguide.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
