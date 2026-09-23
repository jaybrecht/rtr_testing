cmake_minimum_required(VERSION 3.8)
project(rtr_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

set(action_files
  "action/TaskAttempt.action"
)

set(msg_files
  "msg/Assessment.msg"
  "msg/Constraint.msg"
  "msg/ConstraintEvaluation.msg"
  "msg/Exit.msg"
  "msg/Metric.msg"
  "msg/Outcome.msg"
  "msg/SerializedData.msg"
  "msg/TaskSpecification.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}

  DEPENDENCIES
  action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
