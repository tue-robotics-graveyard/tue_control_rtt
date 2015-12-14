#ifndef CONTROLLER_STATE_PUBLISHER_H
#define CONTROLLER_STATE_PUBLISHER_H

#include <tue_control_rtt_msgs/ControllerStates.h>
#include <rtt/OutputPort.hpp>

namespace tue
{

namespace control
{

namespace rtt
{

struct ControllerInfo;

class ControllerStatePublisher
{

public:

    void publish(const std::vector<ControllerInfo>& infos);

    RTT::OutputPort<tue_control_rtt_msgs::ControllerStates>& port() { return port_; }

private:

    RTT::OutputPort<tue_control_rtt_msgs::ControllerStates> port_;

    tue_control_rtt_msgs::ControllerStates message_;

};

}

}

}
#endif
