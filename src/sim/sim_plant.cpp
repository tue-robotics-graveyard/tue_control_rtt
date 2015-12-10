#include "tue/control/rtt/sim/sim_plant.h"

// ORO_CREATE_COMPONENT_MACRO
#include <rtt/Component.hpp>

namespace tue
{

namespace control
{

namespace rtt
{

// ----------------------------------------------------------------------------------------------------

SimPlant::SimPlant(const std::string& name) :
    RTT::TaskContext(name), dt_(0)
{
    addProperty("sampling_time", dt_);
    addProperty("masses", masses_);
    addProperty("positions", positions_);

    addPort("positions", out_port_positions_);
    addPort("efforts", in_port_efforts_);
}

// ----------------------------------------------------------------------------------------------------

SimPlant::~SimPlant()
{
}

// ----------------------------------------------------------------------------------------------------

bool SimPlant::configureHook()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check input properties

    if (dt_ <= 0)
    {
        RTT::log(RTT::Error) << "SimPlant: 'sampling_time' not set or <= 0" << RTT::endlog();
        return false;
    }

    if (masses_.empty())
    {
        RTT::log(RTT::Error) << "Please specify system masses (property 'masses' - array of doubles)" << RTT::endlog();
        return false;
    }


    if (positions_.size() != masses_.size())
    {
        RTT::log(RTT::Error) << "'positions' (array of doubles) not specified, or not same size as 'masses'" << RTT::endlog();
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Initialize systems

    systems_.resize(masses_.size());
    for(unsigned int i = 0; i < systems_.size(); ++i)
    {
        systems_[i].setMass(masses_[i]);
        systems_[i].setPosition(positions_[i]);
    }

    in_efforts_.resize(systems_.size());
    out_positions_.resize(systems_.size());

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool SimPlant::startHook()
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

void SimPlant::updateHook()
{
    if (!in_port_efforts_.read(in_efforts_))
        std::fill(in_efforts_.begin(), in_efforts_.end(), 0);

    for(unsigned int i = 0; i < systems_.size(); ++i)
    {
        SimpleSystem& s = systems_[i];
        s.update(in_efforts_[i], dt_);
    }

    for(unsigned int i = 0; i < systems_.size(); ++i)
    {
        SimpleSystem& s = systems_[i];
        out_positions_[i] = s.position();
    }

    out_port_positions_.write(out_positions_);
}

// ----------------------------------------------------------------------------------------------------

void SimPlant::stopHook()
{
}

// ----------------------------------------------------------------------------------------------------

} // end namespace rtt

} // end namespace control

} // end namespace tue

ORO_CREATE_COMPONENT(tue::control::rtt::SimPlant)
