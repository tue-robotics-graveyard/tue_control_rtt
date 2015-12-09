#include "joint_state_publisher.h"

#include <rtt_roscomm/rtt_rostopic.h>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

void JointStatePublisher::getNamePositionVelocityEffortFromController(const ControllerManager& manager, unsigned int idx, std::string& name, double& position, double& velocity, double& effort)
{
    name = manager.getName(idx);
    position = manager.getMeasurement(idx);
    velocity = 0;
    effort = manager.getOutput(idx);
}

// ----------------------------------------------------------------------------------------------------

void JointStatePublisher::mapMessage(const ControllerManager& manager, sensor_msgs::JointState& message)
{
    unsigned int num_controllers = manager.getNumControllers();

    // Resize if the number of controllers has changed
    if (message.name.size() != num_controllers)
    {
        message.name.resize(num_controllers);
        message.position.resize(num_controllers);
        message.velocity.resize(num_controllers);
        message.effort.resize(num_controllers);
    }

    // Update seq + stamp
    message.header.seq++;
    message.header.stamp = ros::Time::now();

    // Fill the array
    for (unsigned int i = 0; i < num_controllers; ++i)
        getNamePositionVelocityEffortFromController(manager, i, message.name[i], message.position[i], message.velocity[i], message.effort[i]);
}

// ----------------------------------------------------------------------------------------------------

}

}

}
