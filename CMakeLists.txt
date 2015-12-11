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

    tue_manipulation  # For reference generator
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

### Orocos Targets ###

add_library(tue_control_rtt_util
            include/tue/control/rtt/event_clock.h
            src/diagnostics_publisher.cpp
            include/tue/control/rtt/diagnostics_publisher.h
            src/joint_state_publisher.cpp
            include/tue/control/rtt/joint_state_publisher.h
)
target_link_libraries(tue_control_rtt_util ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(controller_manager_component
    src/controller_manager_component.cpp
    include/tue/control/rtt/controller_manager_component.h
)
target_link_libraries(controller_manager_component tue_control_rtt_util)

orocos_component(reference_generator_component
    src/reference_generator_component.cpp
    include/tue/control/rtt/reference_generator_component.h
)
target_link_libraries(reference_generator_component ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

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
