#ifndef CONTROLLER_MANAGER_COMPONENT_H
#define CONTROLLER_MANAGER_COMPONENT_H

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>

#include <tue_control_rtt_msgs/ControllerManagerAction.h>

namespace tue
{

namespace control
{

namespace rtt
{

class ControllerManagerComponent : public RTT::TaskContext
{
public:

    /// Default constructor
    /**
    Constructor for the controller manager component
    */
    ControllerManagerComponent(const std::string& name);

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller manager
    */
    ~ControllerManagerComponent();

protected:
    /**
     * Implement this method such that it contains the code which
     * will be executed when \a configure() is called. The default
     * implementation is an empty function which returns \a true.
     *
     * @retval true to indicate that configuration succeeded and
     * the Stopped state may be entered.
     * @retval false to indicate that configuration failed and the
     * Preoperational state is entered.
     */
    virtual bool configureHook();

    /**
     * Implement this method such that it contains the code which
     * will be executed when \a start() is called. The default implementation is an
     * empty function which returns \a true.
     * @retval true to indicate that the component may run and
     * the Running state may be entered.
     * @retval false to indicate that the component may not run and the
     * Stopped state is entered.
     */
    virtual bool startHook();

    /**
     * Function where the user must insert his 'application' code.
     * When the ExecutionEngine's Activity is a periodic, this
     * function is called by the ExecutionEngine in each periodic
     * step after all messages are processed. When it is executed by a
     * non periodic activity, this function is called after a message
     * is received and executed.  It should not loop
     * forever, since no commands or events are processed when
     * this function executes.  The default implementation is an
     * empty function.
     */
    virtual void updateHook();

    /**
     * Implement this method such that it contains the code which
     * will be executed when \a stop() is called. The default
     * implementation is an empty function.
     */
    virtual void stopHook();

    //! OROCOS I/O

    // ROS outputs
    RTT::OutputPort<diagnostic_msgs::DiagnosticArray> diagnostics_output_port_;
    RTT::OutputPort<sensor_msgs::JointState> joint_states_output_port_;

    // ROS inputs
    RTT::InputPort<tue_control_rtt_msgs::ControllerManagerAction> controller_manager_action_input_port_;

};

}

}

}
#endif
