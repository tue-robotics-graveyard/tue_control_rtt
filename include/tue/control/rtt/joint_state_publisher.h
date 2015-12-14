#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

#include <sensor_msgs/JointState.h>
#include <rtt/OutputPort.hpp>

namespace tue
{

namespace control
{

namespace rtt
{

struct ControllerInfo;

class JointStatePublisher
{

public:

    void publish(const std::vector<ControllerInfo>& infos);

    RTT::OutputPort<sensor_msgs::JointState>& port() { return port_; }

private:

    RTT::OutputPort<sensor_msgs::JointState> port_;

    sensor_msgs::JointState message_;

};

}

}

}
#endif
