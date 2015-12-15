#include "tue/control/rtt/controller_manager_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

#include <rtt_roscomm/rtt_rostopic.h>

#include <tue/config/configuration.h>

#include <tue/control/controller_factory.h>
#include <tue/control/supervised_controller.h>
#include <tue/control/generic_controller.h>
#include <tue/control/setpoint_controller.h>

#include "tue/control/rtt/diagnostics_publisher.h"
#include "tue/control/rtt/joint_state_publisher.h"
#include "tue/control/rtt/controller_state_publisher.h"

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
        connection->data.resize(index + 1, INVALID_DOUBLE);

    return connection;
}

}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::ControllerManagerComponent(const std::string& name) :
    RTT::TaskContext(name),
    dt_(0),
    diagnostics_publisher_(new DiagnosticsPublisher),
    joint_state_publisher_(new JointStatePublisher),
    controller_state_publisher_(new ControllerStatePublisher)
{
    addTopicPort("diagnostics", diagnostics_publisher_->port());
    addTopicPort("joint_states", joint_state_publisher_->port());
    addTopicPort("controller_states", controller_state_publisher_->port());
    addTopicPort("action", in_port_controller_manager_action_);

    addProperty("configuration_rospkg", configuration_rospkg_);
    addProperty("configuration_path", configuration_path_);
    addProperty("sampling_time", dt_);

    // Input ports
    addPort("ref_pos", in_port_ref_positions_);
    addPort("ref_vel", in_port_ref_velocities_);
    addPort("ref_acc", in_port_ref_accelerations_);
    addPort("emergency_switch", in_port_emergency_switch_);

    // Output ports
    addPort("reset_pos", out_port_set_refgen_positions_);

    diagnostics_publisher_clock_.setFrequency(10);
    joint_state_publisher_clock_.setFrequency(10);
    controller_state_publisher_clock_.setFrequency(30);
}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::addTopicPort(const std::string& name, RTT::base::PortInterface& port)
{
    addPort(name, port);
//    port.createStream(rtt_roscomm::topic("~" + this->getName() + "/" + name));
    port.createStream(rtt_roscomm::topic(name));
}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::~ControllerManagerComponent()
{
    delete diagnostics_publisher_;
    delete joint_state_publisher_;
    delete controller_state_publisher_;

    for(std::map<std::string, ControllerInput*>::iterator it = inputs_.begin(); it != inputs_.end(); ++it)
        delete it->second;

    for(std::map<std::string, ControllerOutput*>::iterator it = outputs_.begin(); it != outputs_.end(); ++it)
        delete it->second;
}

// ----------------------------------------------------------------------------------------------------

bool ControllerManagerComponent::configureHook()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check input properties

    if (dt_ <= 0)
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent: 'sampling_time' not set or <= 0" << RTT::endlog();
        return false;
    }

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
    // Create controllers and configure their in and out ports

    ControllerFactory factory;
    factory.registerControllerType<tue::control::GenericController>("generic");
    factory.registerControllerType<tue::control::SetpointController>("setpoint");

    if (config.readArray("controllers"))
    {
        while(config.nextArrayItem())
        {
            std::shared_ptr<tue::control::SupervisedController> c = factory.createController(config, dt_);
            if (!c)
                continue;

            controller_infos_.push_back(ControllerInfo());
            ControllerInfo& io = controller_infos_.back();
            io.controller = c;

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

            // - - - - - - - - - - - - - - - - - - - - - -
            // Add to controller_info map
            controller_info_map_[io.controller->name()] = controller_infos_.size() - 1;
        }
        config.endArray(); // end controllers array
    }

    if (config.hasError())
    {
        RTT::log(RTT::Error) << "ControllerManagerComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int num_joints = controller_infos_.size();

    new_refgen_positions_.resize(num_joints, INVALID_DOUBLE);
    ref_positions_.resize(num_joints, INVALID_DOUBLE);
    ref_velocities_.resize(num_joints, INVALID_DOUBLE);
    ref_accelerations_.resize(num_joints, INVALID_DOUBLE);

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
    // Read emergency switch
    if (in_port_emergency_switch_.read(emergency_switch_) == RTT::NewData)
    {
        RTT::log(RTT::Info) << "Emergency switch input switched to '" << emergency_switch_ << "'" << RTT::endlog();
        for(unsigned int controller_idx = 0; controller_idx < controller_infos_.size(); ++controller_idx)
        {
            if (emergency_switch_)
            {
                controller_infos_[controller_idx].controller->setInactive();
            }
            else
            {
                controller_infos_[controller_idx].controller->setActive();
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read actions
    if (in_port_controller_manager_action_.read(controller_manager_action_) == RTT::NewData)
    {
        for (std::vector<tue_control_rtt_msgs::ControllerAction>::const_iterator it = controller_manager_action_.actions.begin(); it != controller_manager_action_.actions.end(); ++it)
        {
            const tue_control_rtt_msgs::ControllerAction& action = *it;
            std::map<std::string, unsigned int>::iterator controller_info_it = controller_info_map_.find(action.name);
            if (controller_info_it != controller_info_map_.end())
            {
                std::shared_ptr<SupervisedController> supervised_controller = controller_infos_[controller_info_it->second].controller;
                if (action.action == "set_active")
                {
                    supervised_controller->setActive();
                }
                else if (action.action == "set_inactive")
                {
                    supervised_controller->setInactive();
                }
                else
                {
                    RTT::log(RTT::Error) << "Received unknown action '" << action.action << "' for controller '" << action.name << "'!" << RTT::endlog();
                }
            }
            else
            {
                RTT::log(RTT::Error) << "Received action for unknown controller '" << action.name << "'!" << RTT::endlog();
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read measurements

    for(std::map<std::string, ControllerInput*>::iterator it = inputs_.begin(); it != inputs_.end(); ++it)
    {
        ControllerInput* input = it->second;
        input->port.read(input->data);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read references

    in_port_ref_positions_.read(ref_positions_);
    in_port_ref_velocities_.read(ref_velocities_);
    in_port_ref_accelerations_.read(ref_accelerations_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool new_refgen_position = false;

    for(unsigned int controller_idx = 0; controller_idx < controller_infos_.size(); ++controller_idx)
    {
        ControllerInfo& info = controller_infos_[controller_idx];

        // Get measurement
        double measurement = info.input->data[info.input_index];

        // Set reference
        if (info.controller->status() == ACTIVE && is_set(ref_positions_[controller_idx]))
            info.controller->setReference(ref_positions_[controller_idx], ref_velocities_[controller_idx], ref_accelerations_[controller_idx]);

        // Remember status before update
        ControllerStatus old_status = info.controller->status();

        // Update controller
        info.controller->update(measurement);

        // Check if the controller just switched to active
        if (info.controller->status() == ACTIVE && old_status != ACTIVE )
        {
            // Controller just switched to active. Notify reference generator of current position measurement
            new_refgen_positions_[controller_idx] = measurement;
            new_refgen_position = true;

           RTT::log(RTT::Info) << "Controller '" << info.controller->name() << "' (idx = " << controller_idx << ") "
                               << "just got active (current position = " << measurement << ")" << RTT::endlog();
        }

        // Get controller output
        info.output->data[info.output_index] = info.controller->output();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write outputs

    for(std::map<std::string, ControllerOutput*>::iterator it = outputs_.begin(); it != outputs_.end(); ++it)
    {
        ControllerOutput* output = it->second;
        output->port.write(output->data);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // If one of the controllers just switched to active, send the current position to the
    // reference generator

    if (new_refgen_position)
    {
        out_port_set_refgen_positions_.write(new_refgen_positions_);
        std::fill(new_refgen_positions_.begin(), new_refgen_positions_.end(), INVALID_DOUBLE);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (joint_state_publisher_clock_.triggers())
        joint_state_publisher_->publish(controller_infos_);

    if (diagnostics_publisher_clock_.triggers())
        diagnostics_publisher_->publish(controller_infos_);

    if (controller_state_publisher_clock_.triggers())
        controller_state_publisher_->publish(controller_infos_);
}

// ----------------------------------------------------------------------------------------------------

void ControllerManagerComponent::stopHook()
{
}

// ----------------------------------------------------------------------------------------------------

} // end rtt namespace

} // end control namespace

} // end tue namespace

ORO_CREATE_COMPONENT(tue::control::rtt::ControllerManagerComponent)
