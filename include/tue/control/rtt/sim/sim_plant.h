#ifndef CONTROLLER_MANAGER_COMPONENT_H
#define CONTROLLER_MANAGER_COMPONENT_H

//! OROCOS
#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

class SimpleSystem
{

public:

    void setMass(double mass) { mass_ = mass; }

    void setPosition(double pos) { pos_ = pos; vel_ = 0; }

    void update(double f, double dt)
    {
        double a = f / mass_;
        vel_ += dt * a;
        pos_ += dt * vel_;
    }

    double position() const { return pos_; }

private:

    double mass_;
    double pos_;
    double vel_;

};

// ----------------------------------------------------------------------------------------------------

class SimPlant : public RTT::TaskContext
{
public:

    /// Default constructor
    /**
    Constructor for the controller manager component
    */
    SimPlant(const std::string& name);

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller manager
    */
    ~SimPlant();

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

    // Properties
    double dt_;                      // sampling time
    std::vector<double> masses_;     // system masses
    std::vector<double> positions_;  // initial positions


    std::vector<SimpleSystem> systems_;

    std::vector<double> in_efforts_;
    RTT::InputPort<std::vector<double> > in_port_efforts_;

    std::vector<double> out_positions_;
    RTT::OutputPort<std::vector<double> > out_port_positions_;

};

}

}

}
#endif
