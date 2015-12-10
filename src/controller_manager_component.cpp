#include "tue/control/rtt/controller_manager_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

#include <rtt_roscomm/rtt_rostopic.h>

#include <tue/config/configuration.h>

#include "util/diagnostics_publisher.h"
#include "util/joint_state_publisher.h"

#include <ros/package.h>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::ControllerManagerComponent(const std::string& name) :
    RTT::TaskContext(name),
    diagnostics_publisher_(new DiagnosticsPublisher),
    joint_state_publisher_(new JointStatePublisher)
{
    addTopicPort("diagnostics", diagnostics_publisher_->getPort());
    addTopicPort("joint_states", joint_state_publisher_->getPort());
    addTopicPort("action", controller_manager_action_input_port_);

    addProperty("configuration_rospkg", configuration_rospkg_);
    addProperty("configuration_path", configuration_path_);
}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::addTopicPort(const std::string& name, RTT::base::PortInterface& port)
{
    addPort(name, port);
    port.createStream(rtt_roscomm::topic("~" + this->getName() + "/" + name));
}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::~ControllerManagerComponent()
{
    delete diagnostics_publisher_;
    delete joint_state_publisher_;
}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::configureHook()
{
    if (configuration_path_.empty())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): no configuration_path property specified" << RTT::endlog();
        return false;
    }

    std::string config_path;
    if (!configuration_rospkg_.empty() && configuration_path_[0] != '/')
    {
        config_path = ros::package::getPath(configuration_rospkg_);
        if (config_path.empty())
        {
            RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): configuration_rospkg: "
                                 << "No such ros package: '" << configuration_rospkg_ << "'" << RTT::endlog();
            return false;
        }
        config_path += "/";

    }
    config_path += configuration_path_;

    Configuration config;
    config.loadFromYAMLFile(config_path);
    if (config.hasError())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

    manager.configure(config);
    if (config.hasError())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

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
