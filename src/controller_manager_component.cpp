#include "tue/control/rtt/controller_manager_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

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

}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::~ControllerManagerComponent()
{

}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::configureHook()
{

}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::startHook()
{

}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::updateHook()
{

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
