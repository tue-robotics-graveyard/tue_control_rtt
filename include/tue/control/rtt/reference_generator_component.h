#ifndef TUE_CONTROL_RTT_REFERENCE_GENERATOR_COMPONENT_H_
#define TUE_CONTROL_RTT_REFERENCE_GENERATOR_COMPONENT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

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
    // Ouptut

    std::vector<double> ref_positions_;
    std::vector<double> ref_velocities_;
    std::vector<double> ref_accelerations_;

    RTT::OutputPort<std::vector<double> > out_port_ref_positions_;
    RTT::OutputPort<std::vector<double> > out_port_ref_velocities_;
    RTT::OutputPort<std::vector<double> > out_port_ref_accelerations_;

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
