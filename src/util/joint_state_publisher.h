#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

#include <rtt/OutputPort.hpp>
#include <sensor_msgs/JointState.h>

namespace tue
{

namespace control
{

namespace rtt
{

class JointStatePublisher
{
public:

    JointStatePublisher();
    ~JointStatePublisher();

    RTT::OutputPort<sensor_msgs::JointState>& getPort() { return port_; }

protected:
    RTT::OutputPort<sensor_msgs::JointState> port_;

};

}

}

}
#endif
