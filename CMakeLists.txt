cmake_minimum_required(VERSION 2.8.3)
project(tue_control_rtt)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

### Orocos Targets ###

orocos_component(controller_manager_component src/controller_manager_component.cpp include/tue/control/rtt/controller_manager_component.h)
target_link_libraries(controller_manager_component ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/tue/control/rtt)

# Export package information (replaces catkin_package() macro)
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)
