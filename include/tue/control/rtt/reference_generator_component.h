#ifndef TUE_CONTROL_RTT_REFERENCE_GENERATOR_COMPONENT_H_
#define TUE_CONTROL_RTT_REFERENCE_GENERATOR_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <tue/manipulation/reference_generator.h>

#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace tue
{

namespace control
{

namespace rtt
{

class ReferenceGeneratorComponent : public RTT::TaskContext
{

public:

    ReferenceGeneratorComponent(const std::string& name);

    ~ReferenceGeneratorComponent();

private:

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Properties

    std::string configuration_path_;
    std::string configuration_rospkg_;

    double dt_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Input

    std::vector<double> reset_positions_;
    RTT::InputPort<std::vector<double> > in_port_reset_positions_;

    std::vector<double> controller_status_;
    RTT::InputPort<std::vector<double> > in_port_controller_status_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Ouptut

    std::vector<double> ref_positions_;
    std::vector<double> ref_velocities_;
    std::vector<double> ref_accelerations_;

    RTT::OutputPort<std::vector<double> > out_port_ref_positions_;
    RTT::OutputPort<std::vector<double> > out_port_ref_velocities_;
    RTT::OutputPort<std::vector<double> > out_port_ref_accelerations_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Reference generator

    tue::manipulation::ReferenceGenerator reference_generator_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // ROS interface

    ACTION_DEFINITION(control_msgs::FollowJointTrajectoryAction)
    rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> rtt_action_server_;
    typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;

    std::map<std::string, GoalHandle> goal_handles_;

    void goalCallback(GoalHandle gh);

    void cancelCallback(GoalHandle gh);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool configureHook();

    bool startHook();

    void updateHook();

    void stopHook();

};

} // end namespace rtt

} // end namespace control

} // end namespace tue


#endif
