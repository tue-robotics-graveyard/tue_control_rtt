#include "tue/control/rtt/diagnostics_publisher.h"

#include "tue/control/rtt/controller_manager_component.h" // for ControllerInfo

#include <tue/control/supervised_controller.h>

namespace tue
{

namespace control
{

namespace rtt
{

namespace
{

// ----------------------------------------------------------------------------------------------------

template <class T>
inline std::string toString (const T& t){
  std::stringstream ss;
  ss << t;
  return ss.str();
}

// ----------------------------------------------------------------------------------------------------

void controllerDiagnosticsToMessage(const tue::control::SupervisedController& c,
                                    diagnostic_msgs::DiagnosticStatus& status)
{
    if (status.values.size() != 4)
        status.values.resize(4);

    status.values[0].key = "error";
    status.values[0].value = toString(c.error());

    status.values[1].key = "measurement";
    status.values[1].value = toString(c.measurement());

    status.values[2].key = "output";
    status.values[2].value = toString(c.output());

    status.values[3].key = "controller_status";

    status.name = c.name();

    switch(c.status())
    {
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
    case ACTIVE:
        status.level = status.OK;
        status.values[3].value = "ACTIVE";
        break;
    case INACTIVE:
        status.level = status.OK;
        status.values[3].value = "INACTIVE";
        break;
    }

    if (c.status() == tue::control::ERROR)
        status.message = c.error_message();
    else
        status.message.clear();
}

}

// ----------------------------------------------------------------------------------------------------

void DiagnosticsPublisher::publish(const std::vector<ControllerInfo>& infos)
{
    unsigned int num_controllers = infos.size();

    // Resize if the number of controllers has changed
    if (message_.status.size() != num_controllers)
        message_.status.resize(num_controllers);

    // Update seq + stamp
    message_.header.seq++;
    message_.header.stamp = ros::Time::now();

    // Fill the array
    for (unsigned int i = 0; i < num_controllers; ++i)
        controllerDiagnosticsToMessage(*infos[i].controller, message_.status[i]);

    port_.write(message_);
}

// ----------------------------------------------------------------------------------------------------

}

}

}
