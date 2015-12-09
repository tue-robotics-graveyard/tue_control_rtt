#ifndef DIAGNOSTICS_PUBLISHER_H
#define DIAGNOSTICS_PUBLISHER_H

#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace tue
{

namespace control
{

namespace rtt
{

class DiagnosticsPublisher
{
public:

    DiagnosticsPublisher();
    ~DiagnosticsPublisher();

    RTT::OutputPort<diagnostic_msgs::DiagnosticArray>& getPort() { return port_; }

protected:
    std::string name_;
    RTT::OutputPort<diagnostic_msgs::DiagnosticArray> port_;

};

}

}

}
#endif
