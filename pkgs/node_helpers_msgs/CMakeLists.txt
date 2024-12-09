cmake_minimum_required(VERSION 3.5)
project(node_helpers_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

######################################################################
########## NOTE TO TEMPLATE USERS: When adding messages that require other messages,
########## add the following line below to ensure that the other messages are found.
########## Also make sure new messages are added to the rosidl_generate_interfaces list.
# find_package(my_required_msgs_package REQUIRED)
######################################################################


rosidl_generate_interfaces(
  ${PROJECT_NAME}

  ######################################################################
  ########## NOTE TO TEMPLATE USERS: Add new messages here, like so:
  # msg/ANewMessageFile.msg
  ######################################################################

  "msg/BinaryReading.msg"
  "msg/PlaySound.msg"
  "msg/PromptOption.msg"
  "msg/RangefinderReading.msg"
  "msg/SensorExample.msg"
  "msg/UserPrompt.msg"

  "action/RobustActionExample.action"

  "srv/ChoosePromptOption.srv"
  "srv/RobustServiceExample.srv"

  ######################################################################
  ########## NOTE TO TEMPLATE USERS: Add your dependency packages here, like so:
  # DEPENDENCIES my_required_msgs_package
  ######################################################################
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
