#ifndef TUE_CONTROL_RTT_DIAGNOSTICS_PUBLISHER_H
#define TUE_CONTROL_RTT_DIAGNOSTICS_PUBLISHER_H

#include <diagnostic_msgs/DiagnosticArray.h>
#include <rtt/OutputPort.hpp>

namespace tue
{

namespace control
{

namespace rtt
{

struct ControllerInfo;

class DiagnosticsPublisher
{

public:

    void publish(const std::vector<ControllerInfo>& infos);

    RTT::OutputPort<diagnostic_msgs::DiagnosticArray>& port() { return port_; }

private:

    RTT::OutputPort<diagnostic_msgs::DiagnosticArray> port_;

    diagnostic_msgs::DiagnosticArray message_;

};

}

}

}
#endif
