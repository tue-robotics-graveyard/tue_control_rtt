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

namespace
{

template<typename T>
T* addOrUpdateConnection(ControllerManagerComponent* compoment, std::map<std::string, T*>& connections, tue::Configuration& config)
{
    // A connection should always have a "port" and "index"
    int index;
    std::string port_name;
    if (!config.value("port", port_name) | !config.value("index", index))
        return NULL;

    T* connection;
    typename std::map<std::string, T*>::iterator it = connections.find(port_name);
    if (it == connections.end())
    {
        connection = new T;
        compoment->addPort(port_name, connection->port);
        connections[port_name] = connection;
    }
    else
    {
        connection = it->second;
    }

    if (index + 1 > connection->data.size())
        connection->data.resize(index + 1, 0);

    return connection;
}

}

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

    for(std::map<std::string, ControllerInput*>::iterator it = inputs_.begin(); it != inputs_.end(); ++it)
        delete it->second;

    for(std::map<std::string, ControllerOutput*>::iterator it = outputs_.begin(); it != outputs_.end(); ++it)
        delete it->second;
}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::configureHook()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine configuration file path

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Load configuration file

    Configuration config;
    config.loadFromYAMLFile(config_path);
    if (config.hasError())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure manager

    manager_.configure(config);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure in and out ports

    controller_ios_.resize(manager_.getNumControllers());

    if (config.readArray("controllers"))
    {
        while(config.nextArrayItem())
        {
            std::string controller_name;
            if (!config.value("name", controller_name))
                continue;

            int controller_idx = manager_.getControllerIdx(controller_name);
            if (controller_idx < 0)
                continue;

            ControllerIO& io = controller_ios_[controller_idx];

            // - - - - - - - - - - - - - - - - - - - - - -
            // Configure in port

            if (config.readGroup("input", tue::REQUIRED))
            {
                io.input = addOrUpdateConnection(this, inputs_, config);
                config.value("index", io.input_index);
                config.endGroup(); // end group 'input'
            }

            // - - - - - - - - - - - - - - - - - - - - - -
            // Configure out port

            if (config.readGroup("output", tue::REQUIRED))
            {
                io.output = addOrUpdateConnection(this, outputs_, config);
                config.value("index", io.output_index);
                config.endGroup(); // end group 'output'
            }

        }
        config.endArray();
    }

    if (config.hasError())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

    diagnostics_publisher_->configure(manager_, 10);
    joint_state_publisher_->configure(manager_, 10);

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
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read measurements

    for(std::map<std::string, ControllerInput*>::iterator it = inputs_.begin(); it != inputs_.end(); ++it)
    {
        ControllerInput* input = it->second;
        input->port.read(input->data);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set references and measurements to the controllers

    for(unsigned int controller_idx = 0; controller_idx < controller_ios_.size(); ++controller_idx)
    {
        ControllerIO& io = controller_ios_[controller_idx];

        // Set measurement
        double measurement = io.input->data[io.input_index];
        manager_.setMeasurement(controller_idx, measurement);

        // Set reference
        manager_.setReference(controller_idx, 0);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update the controllers

    manager_.update();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Get outputs from controllers

    for(unsigned int controller_idx = 0; controller_idx < controller_ios_.size(); ++controller_idx)
    {
        ControllerIO& io = controller_ios_[controller_idx];

        double output = manager_.getOutput(controller_idx);
        io.output->data[io.output_index] = output;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write outputs

    for(std::map<std::string, ControllerOutput*>::iterator it = outputs_.begin(); it != outputs_.end(); ++it)
    {
        ControllerOutput* output = it->second;
        output->port.write(output->data);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    joint_state_publisher_->publish();
    diagnostics_publisher_->publish();
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
