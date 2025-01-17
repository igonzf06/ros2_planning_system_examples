cmake_minimum_required(VERSION 3.8)
project(plansys2_doorbell_attend_example)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(text_to_speech_interfaces REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    text_to_speech_interfaces
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
add_executable(checkdoor_action_node src/checkdoor_action_node.cpp)
ament_target_dependencies(checkdoor_action_node ${dependencies})
add_executable(doorcheck_controller_node src/doorcheck_controller_node.cpp)
ament_target_dependencies(doorcheck_controller_node ${dependencies})
add_executable(listen_action_node src/listen_action_node.cpp)
ament_target_dependencies(listen_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  checkdoor_action_node
  doorcheck_controller_node
  listen_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
