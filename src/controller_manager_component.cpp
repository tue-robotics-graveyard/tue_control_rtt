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
        connection->data.resize(index + 1, INVALID_DOUBLE);

    return connection;
}

}

// ----------------------------------------------------------------------------------------------------

ControllerManagerComponent::ControllerManagerComponent(const std::string& name) :
    RTT::TaskContext(name),
    dt_(0),
    diagnostics_publisher_(new DiagnosticsPublisher),
    joint_state_publisher_(new JointStatePublisher)
{
    addTopicPort("diagnostics", diagnostics_publisher_->getPort());
    addTopicPort("joint_states", joint_state_publisher_->getPort());
    addTopicPort("action", controller_manager_action_input_port_);

    addProperty("configuration_rospkg", configuration_rospkg_);
    addProperty("configuration_path", configuration_path_);
    addProperty("sampling_time", dt_);

    // Input ports
    addPort("ref_pos", in_port_ref_positions_);
    addPort("ref_vel", in_port_ref_velocities_);
    addPort("ref_acc", in_port_ref_accelerations_);

    // Output ports
    addPort("reset_pos", out_port_set_refgen_positions_);
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
    // Configure manager

    config.setValue("dt", dt_);
    manager_.configure(config);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure in and out ports

    controller_infos_.resize(manager_.getNumControllers());

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

            ControllerInfo& io = controller_infos_[controller_idx];

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int num_joints = manager_.getNumControllers();

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
    // Set references and measurements to the controllers

    for(unsigned int controller_idx = 0; controller_idx < controller_infos_.size(); ++controller_idx)
    {
        ControllerInfo& info = controller_infos_[controller_idx];

        // Set measurement
        double measurement = info.input->data[info.input_index];
        manager_.setMeasurement(controller_idx, measurement);

        // Set reference
        if (info.status == READY && is_set(ref_positions_[controller_idx]))
        {
            manager_.setReference(controller_idx, ref_positions_[controller_idx], ref_velocities_[controller_idx], ref_accelerations_[controller_idx]);
        }
        else
        {
            manager_.setReference(controller_idx, measurement, 0, 0);
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update the controllers

    manager_.update();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Get outputs from controllers

    for(unsigned int controller_idx = 0; controller_idx < controller_infos_.size(); ++controller_idx)
    {
        ControllerInfo& info = controller_infos_[controller_idx];
        double output = manager_.getOutput(controller_idx);
        info.output->data[info.output_index] = output;

    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write outputs

    for(std::map<std::string, ControllerOutput*>::iterator it = outputs_.begin(); it != outputs_.end(); ++it)
    {
        ControllerOutput* output = it->second;
        output->port.write(output->data);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check if controllers change from non-ready to ready. If so, notify the reference
    // generator of the current position of this controller

    bool new_refgen_position = false;
    for(unsigned int controller_idx = 0; controller_idx < controller_infos_.size(); ++controller_idx)
    {
        ControllerInfo& info = controller_infos_[controller_idx];

        ControllerStatus old_status = info.status;
        ControllerStatus new_status = manager_.getStatus(controller_idx);

        if (old_status != tue::control::READY && new_status == READY)
        {
            double measurement = manager_.getMeasurement(controller_idx);
            if (is_set(measurement))
            {
                std::cout << "Controller " << controller_idx << " just got ready (current position = " << measurement << ")" << std::endl;

                // Controller just switched to ready. Notify reference generator of current position measurement
                new_refgen_positions_[controller_idx] = measurement;
                new_refgen_position = true;

                info.status = new_status;
            }
        }
        else
        {
            info.status = new_status;
        }
    }

    if (new_refgen_position)
    {
        out_port_set_refgen_positions_.write(new_refgen_positions_);
        std::fill(new_refgen_positions_.begin(), new_refgen_positions_.end(), INVALID_DOUBLE);
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

} // end rtt namespace

} // end control namespace

} // end tue namespace

ORO_CREATE_COMPONENT(tue::control::rtt::ControllerManagerComponent)
