#include "diagnostics_publisher.h"

template <class T>
inline std::string toString (const T& t){
  std::stringstream ss;
  ss << t;
  return ss.str();
}

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

void DiagnosticsPublisher::getDiagnosticsStatusFromController(const ControllerManager& manager, unsigned int idx, diagnostic_msgs::DiagnosticStatus& status)
{
    if (status.values.size() != 4)
        status.values.resize(4);

    status.values[0].key = "error";
    status.values[0].value = toString(manager.getError(idx));

    status.values[1].key = "measurement";
    status.values[1].value = toString(manager.getMeasurement(idx));

    status.values[2].key = "output";
    status.values[2].value = toString(manager.getOutput(idx));

    status.values[3].key = "controller_status";

    status.name = manager.getName(idx);

    switch(manager.getStatus(idx)) {
        case ERROR:
            status.level = status.ERROR;
            status.values[3].value = "ERROR";
            break;
        case UNINITIALIZED:
            status.level = status.STALE;
            status.values[3].value = "UNINITIALIZED";
            break;
        case HOMING:
            status.level = status.WARN;
            status.values[3].value = "HOMING";
            break;
        case READY:
            status.level = status.OK;
            status.values[3].value = "READY";
            break;
    }
}

// ----------------------------------------------------------------------------------------------------

void DiagnosticsPublisher::mapMessage(const ControllerManager& manager, diagnostic_msgs::DiagnosticArray& message)
{
    unsigned int num_controllers = manager.getNumControllers();

    // Resize if the number of controllers has changed
    if (message.status.size() != num_controllers)
        message.status.resize(num_controllers);

    // Update seq + stamp
    message.header.seq++;
    message.header.stamp = ros::Time::now();

    // Fill the array
    for (unsigned int i = 0; i < num_controllers; ++i)
        getDiagnosticsStatusFromController(manager, i, message.status[i]);
}

// ----------------------------------------------------------------------------------------------------

}

}

}
