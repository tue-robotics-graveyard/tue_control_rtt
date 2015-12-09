#include "tue/control/rtt/controller_manager_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

#include <rtt_roscomm/rtt_rostopic.h>

#include <string>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::ControllerManagerComponent(const std::string& name) :
    RTT::TaskContext(name)
{
    //! Setup diagnostics topic
    std::string diagnostics_name = "diagnostics";
    addPort(diagnostics_name, diagnostics_output_port_);
    diagnostics_output_port_.createStream(rtt_roscomm::topic("~" + this->getName() + "/" + diagnostics_name));

    //! Setup joint states topic
    std::string joint_states_name = "joint_states";
    addPort(joint_states_name, joint_states_output_port_);
    joint_states_output_port_.createStream(rtt_roscomm::topic("~" + this->getName() + "/" + joint_states_name));

    //! Setup controller manager action input topic
    std::string controller_manager_action_name = "action";
    addPort(controller_manager_action_name, controller_manager_action_input_port_);
    controller_manager_action_input_port_.createStream(rtt_roscomm::topic("~" + this->getName() + "/" + controller_manager_action_name));
}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::~ControllerManagerComponent()
{

}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::configureHook()
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::startHook()
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::updateHook()
{
    diagnostic_msgs::DiagnosticArray diagnostics_array;
    diagnostics_output_port_.write(diagnostics_array);
}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::stopHook()
{

}

// ----------------------------------------------------------------------------------------------------

}

}

}

ORO_CREATE_COMPONENT(tue::control::rtt::ControllerManagerComponent)
