#include "LocalTask.hpp"

#include <orocos_cpp/orocos_cpp.hpp>
#include <orocos_cpp_base/OrocosHelpers.hpp>

LocalTask::LocalTask(const std::string &name) {
    orocos_cpp::OrocosCppConfig config;

    config.load_all_packages = true;
    config.load_typekits = true; //TODO: only load base-types 
    // config.corba_host = host;

    orocos = std::make_shared<orocos_cpp::OrocosCpp>();
    orocos->initialize(config, true);
    OrocosHelpers::initClientTask(name);
    task = OrocosHelpers::getClientTask();

    
    sample_port = std::make_shared<RTT::InputPort<base::samples::Joints>>("samples");
    task->ports()->addPort(sample_port->getName(), *sample_port );    
    control_port = std::make_shared<RTT::OutputPort<base::samples::Joints>>("control");
    task->ports()->addPort(control_port->getName(), *control_port );
    task->start();

}

LocalTask::~LocalTask(){
    task->stop();
}

bool LocalTask::connectSamples(RTT::base::OutputPortInterface* remotePort) {
    remotePort->connectTo(sample_port.get());
}
bool LocalTask::connectCommands(RTT::base::InputPortInterface* remotePort) {
    control_port->connectTo(remotePort);
}
