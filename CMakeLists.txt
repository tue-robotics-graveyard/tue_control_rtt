cmake_minimum_required(VERSION 2.8.3)
project(tue_control_rtt)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    rtt_roscomm
    diagnostic_msgs
    sensor_msgs
    tue_control_rtt_msgs
    tue_control
    tue_config
    roslib  # Used for find package
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

### Orocos Targets ###

add_library(tue_control_rtt_util
            src/util/event_clock.h
            src/util/publisher.h
            src/util/diagnostics_publisher.cpp
            src/util/diagnostics_publisher.h
            src/util/joint_state_publisher.cpp
            src/util/joint_state_publisher.h
)
target_link_libraries(tue_control_rtt_util ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(controller_manager_component src/controller_manager_component.cpp include/tue/control/rtt/controller_manager_component.h)
target_link_libraries(controller_manager_component tue_control_rtt_util)

# --------------------------------------------------------------------------------
#                                 SIMULATION
# --------------------------------------------------------------------------------

orocos_component(sim_plant_component src/sim/sim_plant.cpp include/tue/control/rtt/sim/sim_plant.h)
target_link_libraries(controller_manager_component tue_control_rtt_util)


### Orocos Package Exports and Install Targets ###

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/tue/control/rtt)

# Export package information (replaces catkin_package() macro)
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)
