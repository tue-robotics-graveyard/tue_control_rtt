#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

#include "publisher.h"
#include <sensor_msgs/JointState.h>

namespace tue
{

namespace control
{

namespace rtt
{

class JointStatePublisher : public Publisher<sensor_msgs::JointState>
{
protected:
    virtual void mapMessage(const ControllerManager& manager, sensor_msgs::JointState& message);
    void getNamePositionVelocityEffortFromController(const ControllerManager& manager, unsigned int idx, std::string& name, double& position, double& velocity, double& effort);

};

}

}

}
#endif
