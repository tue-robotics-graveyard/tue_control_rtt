#include "tue/control/rtt/joint_state_publisher.h"
#include "tue/control/rtt/controller_manager_component.h" // for ControllerInfo

#include <tue/control/supervised_controller.h>

#include <rtt_roscomm/rtt_rostopic.h>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

void JointStatePublisher::publish(const std::vector<ControllerInfo>& infos)
{
    unsigned int num_controllers = infos.size();

    // Resize if the number of controllers has changed
    if (message_.name.size() != num_controllers)
    {
        message_.name.resize(num_controllers);
        message_.position.resize(num_controllers);
        message_.velocity.resize(num_controllers);
        message_.effort.resize(num_controllers);
    }

    // Update seq + stamp
    message_.header.seq++;
    message_.header.stamp = ros::Time::now();

    for(unsigned int i = 0; i < num_controllers; ++i)
    {
        std::shared_ptr<tue::control::SupervisedController> c = infos[i].controller;
        message_.name[i] = c->name();
        message_.position[i] = c->measurement();
        message_.velocity[i] = 0;
        message_.effort[i] = 0;
    }

    port_.write(message_);
}

// ----------------------------------------------------------------------------------------------------

}

}

}
