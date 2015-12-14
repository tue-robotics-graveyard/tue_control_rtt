#include "tue/control/rtt/reference_generator_component.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

#include <tue/config/configuration.h>

#include <ros/package.h>

#include <tue/control/generic.h>

#include <urdf/model.h>

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

    // Input port
    addPort("reset_pos", in_port_reset_positions_);

    // Output ports
    addPort("ref_pos", out_port_ref_positions_);
    addPort("ref_vel", out_port_ref_velocities_);
    addPort("ref_acc", out_port_ref_accelerations_);

    // Add action server ports to this task's root service
    rtt_action_server_.addPorts(this->provides());

    // Bind action server goal and cancel callbacks
    rtt_action_server_.registerGoalCallback(boost::bind(&ReferenceGeneratorComponent::goalCallback, this, _1));
    rtt_action_server_.registerCancelCallback(boost::bind(&ReferenceGeneratorComponent::cancelCallback, this, _1));
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
    // Register joints in reference generator

    if (config.readArray("controllers"))
    {
        while(config.nextArrayItem())
        {
            std::string controller_name;
            if (!config.value("name", controller_name))
                continue;

            reference_generator_.initJoint(controller_name, 0, 0, 0, 0);
        }

        config.endArray(); // end array controllers
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure joint limits in reference generator

    if (config.readGroup("reference_generator"))
    {
        std::string urdf_filename;
        config.value("urdf_file", urdf_filename);
        std::cout << "urdf: " << urdf_filename;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        urdf::Model model;
        model.initFile(urdf_filename);

        const std::vector<std::string>& joint_names = reference_generator_.joint_names();
        for(unsigned int i = 0 ; i < joint_names.size(); ++i)
        {
            boost::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_names[i]);
            reference_generator_.setPositionLimits(i, joint->limits->lower, joint->limits->upper);
            reference_generator_.setMaxVelocity(i, joint->limits->velocity);
            reference_generator_.setMaxAcceleration(i, joint->limits->effort);
        }

        config.endGroup(); // end reference_generator group
    }

    unsigned int num_joints = reference_generator_.joint_names().size();
    reset_positions_.resize(num_joints, tue::control::INVALID_DOUBLE);
    ref_positions_.resize(num_joints, tue::control::INVALID_DOUBLE);
    ref_velocities_.resize(num_joints, tue::control::INVALID_DOUBLE);
    ref_accelerations_.resize(num_joints, tue::control::INVALID_DOUBLE);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGeneratorComponent::startHook()
{
    // Start the actionlib server
    rtt_action_server_.start();

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGeneratorComponent::updateHook()
{
//    RTT::log(RTT::Error) << "ReferenceGeneratorComponent::updateHook()" << RTT::endlog();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Reset reference generator if asked

    if (in_port_reset_positions_.read(reset_positions_) == RTT::NewData)
    {
        for(unsigned int i = 0; i < reset_positions_.size(); ++i)
        {
            double pos = reset_positions_[i];
            if (!is_set(pos))
                continue;

            RTT::log(RTT::Info) << "Setting reference generator state of joint " << i << " to " << pos << RTT::endlog();

            reference_generator_.setJointState(i, pos, 0);
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // If not active goals, do nothing

    if (!reference_generator_.hasActiveGoals())
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Generate references

    reference_generator_.calculatePositionReferences(dt_, ref_positions_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check for each goal if it succeeded or canceled, and notify the goal handle accordingly

    for(std::map<std::string, GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
    {
        GoalHandle& gh = it->second;
        tue::manipulation::JointGoalStatus status = reference_generator_.getGoalStatus(gh.getGoalID().id);

        if (status == tue::manipulation::JOINT_GOAL_SUCCEEDED)
        {
            gh.setSucceeded();
            goal_handles_.erase(it++);
        }
        else if (status == tue::manipulation::JOINT_GOAL_CANCELED)
        {
            gh.setCanceled();
            goal_handles_.erase(it++);
        }
        else
        {
            ++it;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Fill output data

    for(unsigned int i = 0; i < ref_positions_.size(); ++i)
    {
        if (!reference_generator_.joint_state(i).is_set)
        {
            ref_positions_[i] = INVALID_DOUBLE;
            continue;
        }

        ref_velocities_[i] = reference_generator_.joint_state(i).velocity();
        ref_accelerations_[i] = reference_generator_.joint_state(i).acceleration();
    }

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

void ReferenceGeneratorComponent::goalCallback(GoalHandle gh)
{
    RTT::log(RTT::Info) << "ReferenceGeneratorComponent: Received goal" << RTT::endlog();

    std::stringstream error;
    std::string goalid = gh.getGoalID().id;
    if (!reference_generator_.setGoal(*gh.getGoal(), goalid, error))
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_string = error.str();
        result.error_code = 1;

        gh.setRejected(result);
        ROS_ERROR("%s", result.error_string.c_str());
        return;
    }

    // Accept the goal
    gh.setAccepted();
    goal_handles_[goalid] = gh;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGeneratorComponent::cancelCallback(GoalHandle gh)
{
    gh.setCanceled();
    reference_generator_.cancelGoal(gh.getGoalID().id);
    goal_handles_.erase(gh.getGoalID().id);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace rtt

} // end namespace control

} // end namespace tue

ORO_CREATE_COMPONENT(tue::control::rtt::ReferenceGeneratorComponent)
