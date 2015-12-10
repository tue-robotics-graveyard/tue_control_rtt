#ifndef CONTROLLER_MANAGER_COMPONENT_H
#define CONTROLLER_MANAGER_COMPONENT_H

//! OROCOS
#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

//! ROS
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>
#include <tue_control_rtt_msgs/ControllerManagerAction.h>

//! CONTROL
#include <tue/control/controller_manager.h>

namespace tue
{

namespace control
{

namespace rtt
{

class DiagnosticsPublisher;
class JointStatePublisher;

// ----------------------------------------------------------------------------------------------------

struct ControllerInput
{
    RTT::InputPort<std::vector<double> > port;
    std::vector<double> data;
};

// ----------------------------------------------------------------------------------------------------

struct ControllerOutput
{
    RTT::OutputPort<std::vector<double> > port;
    std::vector<double> data;
};

// ----------------------------------------------------------------------------------------------------

struct ControllerIO
{
    ControllerIO() : input(NULL), output(NULL) {}

    ControllerInput* input;
    int input_index;

    ControllerOutput* output;
    int output_index;
};

// ----------------------------------------------------------------------------------------------------

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Properties

    std::string configuration_path_;
    std::string configuration_rospkg_;

    double dt_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    //! Controller manager
    ControllerManager manager_;

    //! OROCOS ROS I/O
    void addTopicPort(const std::string& name, RTT::base::PortInterface& port);

    //! Dashboard actions input port
    RTT::InputPort<tue_control_rtt_msgs::ControllerManagerAction> controller_manager_action_input_port_;

    //! Diagnostics publisher
    DiagnosticsPublisher* diagnostics_publisher_;

    //! Joint state publisher
    JointStatePublisher* joint_state_publisher_;


    std::vector<ControllerIO> controller_ios_;
    std::map<std::string, ControllerInput*> inputs_;
    std::map<std::string, ControllerOutput*> outputs_;

};

}

}

}
#endif
