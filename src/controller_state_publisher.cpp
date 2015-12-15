#include "tue/control/rtt/controller_state_publisher.h"
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

void ControllerStatePublisher::publish(const std::vector<ControllerInfo>& infos)
{
    unsigned int num_controllers = infos.size();

    // Resize if the number of controllers has changed
    if (message_.states.size() != num_controllers)
        message_.states.resize(num_controllers);

//    // Update seq + stamp
//    message_.header.seq++;
//    message_.header.stamp = ros::Time::now();

    message_.timestamp = ros::Time::now();

    for(unsigned int i = 0; i < num_controllers; ++i)
    {
        std::shared_ptr<tue::control::SupervisedController> c = infos[i].controller;
        tue_control_rtt_msgs::ControllerState& s = message_.states[i];
        s.measurement = c->measurement();
        s.output = c->output();
        s.ref_position = c->reference_position();
        s.ref_velocity = c->reference_velocity();
        s.ref_acceleration = c->reference_acceleration();
        s.status = c->status();
    }

    port_.write(message_);
}

// ----------------------------------------------------------------------------------------------------

}

}

}
