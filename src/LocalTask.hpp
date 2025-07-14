#pragma once

#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>

#include <string>
#include <memory>

#include <base/samples/Joints.hpp>


namespace orocos_cpp{
    class OrocosCpp;
}
namespace RTT{
    class TaskContext;
    namespace base {
        class OutputPortInterface;
        class InputPortInterface;
    }
}

class LocalTask {
 public:
    LocalTask(const std::string &name);
    virtual ~LocalTask();
    
    bool connectSamples(RTT::base::OutputPortInterface* remotePort);
    bool connectCommands(RTT::base::InputPortInterface* remotePort);
                                 
    bool getSample(base::samples::Joints * joints){
        if (sample_port->readNewest(*joints) == RTT::NewData) {
            return true;
        }
        return false;
    }
    void sendCommand(const base::samples::Joints& cmd) {
        control_port->write(cmd);
    }
    

 private:
    std::shared_ptr<orocos_cpp::OrocosCpp> orocos;
    RTT::TaskContext *task;

    std::shared_ptr<RTT::InputPort<base::samples::Joints>> sample_port;
    std::shared_ptr<RTT::OutputPort<base::samples::Joints>> control_port;

};
