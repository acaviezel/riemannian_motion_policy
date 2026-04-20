cmake_minimum_required(VERSION 3.5)
project(riemannian_motion_policy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(generate_parameter_library)
find_package(std_msgs REQUIRED)
find_package(messages_fr3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)



add_library(
        ${PROJECT_NAME}
        SHARED
        src/riemannian_motion_policy.cpp
        src/config.yaml
        src/user_input_server.cpp
        src/friction_compensation.cpp
        src/gravity_compensation.cpp
        
        )
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        ament_index_cpp
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        franka_semantic_components
        std_msgs
        messages_fr3
        franka_msgs
        visualization_msgs
        moveit_ros_planning_interface
        moveit_visual_tools
)


pluginlib_export_plugin_description_file(
        controller_interface riemannian_motion_policy.xml)

add_executable(user_input_client src/user_input_client.cpp)
target_link_libraries(user_input_client ${PROJECT_NAME})




install(TARGETS
        user_input_client
        DESTINATION lib/${PROJECT_NAME})

install(
        TARGETS 
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
        FILES src/config.yaml
        DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)
    find_package(moveit_ros_planning_interface REQUIRED)


    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        ament_index_cpp
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        messages_fr3
        franka_msgs
)
ament_package()
