#ifndef TUE_CONTROL_RTT_PUBLISHER_H
#define TUE_CONTROL_RTT_PUBLISHER_H

#include <rtt/OutputPort.hpp>
#include <tue/control/controller_manager.h>

#include "event_clock.h"

namespace tue
{

namespace control
{

namespace rtt
{

template <class T>
class Publisher
{
public:

    Publisher() : configured_(false) {}
    ~Publisher()
    {
        if (configured_)
            delete event_clock_;
    }

    bool configure(const ControllerManager& manager, double frequency)
    {
        if (configured_)
            return false;

        manager_ = &manager;
        event_clock_ = new EventClock(frequency);

        configured_ = true;

        return true;
    }

    bool publish()
    {
        if (!configured_)
            return false;

        if (!event_clock_->triggers())
            return true;

        mapMessage(*manager_, message_);

        port_.write(message_);

        return true;
    }

    RTT::OutputPort<T>& getPort() { return port_; }

private:
    std::string name_;

    T message_;
    RTT::OutputPort<T> port_;

    bool configured_;

    const ControllerManager* manager_;
    EventClock* event_clock_;

protected:
    virtual void mapMessage(const ControllerManager& manager, T& message_) = 0;

};

}

}

}
#endif
