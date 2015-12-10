#ifndef TUE_CONTROL_RTT_DIAGNOSTICS_PUBLISHER_H
#define TUE_CONTROL_RTT_DIAGNOSTICS_PUBLISHER_H

#include "publisher.h"
#include <diagnostic_msgs/DiagnosticArray.h>

namespace tue
{

namespace control
{

namespace rtt
{

class DiagnosticsPublisher : public Publisher<diagnostic_msgs::DiagnosticArray>
{

protected:
    virtual void mapMessage(const ControllerManager& manager, diagnostic_msgs::DiagnosticArray& message);
    void getDiagnosticsStatusFromController(const ControllerManager &manager, unsigned int idx, diagnostic_msgs::DiagnosticStatus& status);

};

}

}

}
#endif
