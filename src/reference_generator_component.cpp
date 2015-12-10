#include "tue/control/rtt/reference_generator_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

#include <tue/config/configuration.h>

#include <ros/package.h>

#include <tue/control/generic.h>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

ReferenceGeneratorComponent::ReferenceGeneratorComponent(const std::string& name) :
    RTT::TaskContext(name), dt_(0)
{
    addProperty("configuration_rospkg", configuration_rospkg_);
    addProperty("configuration_path", configuration_path_);
    addProperty("sampling_time", dt_);

    // Output ports
    addPort("ref_pos", out_port_ref_positions_);
    addPort("ref_vel", out_port_ref_velocities_);
    addPort("ref_acc", out_port_ref_accelerations_);
}

// ----------------------------------------------------------------------------------------------------

ReferenceGeneratorComponent::~ReferenceGeneratorComponent()
{
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGeneratorComponent::configureHook()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check input properties

    if (dt_ <= 0)
    {
        RTT::log(RTT::Error) << "ReferenceGeneratorComponent: 'sampling_time' not set or <= 0" << RTT::endlog();
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine configuration file path

    if (configuration_path_.empty())
    {
        RTT::log(RTT::Error) << "ReferenceGeneratorComponent::configureHook(): no configuration_path property specified" << RTT::endlog();
        return false;
    }

    std::string config_path;
    if (!configuration_rospkg_.empty() && configuration_path_[0] != '/')
    {
        config_path = ros::package::getPath(configuration_rospkg_);
        if (config_path.empty())
        {
            RTT::log(RTT::Error) << "ReferenceGeneratorComponent::configureHook(): configuration_rospkg: "
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
        RTT::log(RTT::Error) << "ReferenceGeneratorComponent::configureHook(): " << config.error() << RTT::endlog();
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int num_controllers = 0;
    if (config.readArray("controllers"))
    {
        while(config.nextArrayItem())
        {
            std::string controller_name;
            if (!config.value("name", controller_name))
                continue;

            ++num_controllers;

//            std::cout << "RefGen: " << controller_name << std::endl;
        }

        config.endArray(); // end array controllers
    }

    ref_positions_.resize(num_controllers, tue::control::INVALID_DOUBLE);
    ref_velocities_.resize(num_controllers, tue::control::INVALID_DOUBLE);
    ref_accelerations_.resize(num_controllers, tue::control::INVALID_DOUBLE);

}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGeneratorComponent::startHook()
{

}

// ----------------------------------------------------------------------------------------------------

void ReferenceGeneratorComponent::updateHook()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write output references (pos, vel and acc)

    out_port_ref_positions_.write(ref_positions_);
    out_port_ref_velocities_.write(ref_velocities_);
    out_port_ref_accelerations_.write(ref_accelerations_);
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGeneratorComponent::stopHook()
{

}

// ----------------------------------------------------------------------------------------------------

} // end namespace rtt

} // end namespace control

} // end namespace tue

ORO_CREATE_COMPONENT(tue::control::rtt::ReferenceGeneratorComponent)
