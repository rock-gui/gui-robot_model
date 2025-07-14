#include "RemoteTasks.hpp"


#include <orocos_cpp/CorbaNameService.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/InputPortInterface.hpp>
#include <rtt/base/OutputPortInterface.hpp>

RemoteTasks::RemoteTasks() {
    ns = std::make_unique<orocos_cpp::CorbaNameService>();
}

bool RemoteTasks::connect() {
    return ns->connect();
}

std::vector<std::string> RemoteTasks::getTasks() {
    return ns->getRegisteredTasks();
}

RTT::TaskContext* RemoteTasks::getTaskContext(const std::string &taskname) {
    return ns->getTaskContext(taskname);
}

RTT::base::OutputPortInterface* RemoteTasks::getOutputPort(const std::string &taskname, const std::string &portname) {
    return dynamic_cast<RTT::base::OutputPortInterface*>(getTaskContext(taskname)->getPort(portname));
}

RTT::base::InputPortInterface* RemoteTasks::getInputPort(const std::string &taskname, const std::string &portname) {
    return dynamic_cast<RTT::base::InputPortInterface*>(getTaskContext(taskname)->getPort(portname));
}

std::map<std::string, std::vector<std::string>> RemoteTasks::getOutputPortsByType(const std::string &typestr) {
    std::map<std::string, std::vector<std::string>> result;

    std::vector<std::string> tasks = ns->getRegisteredTasks();
    for (const std::string &tname : tasks) {
        result[tname] = getTaskOutputPortsByType(tname, typestr);
    }
    return result;
}

std::map<std::string, std::vector<std::string>> RemoteTasks::getInputPortsByType(const std::string &typestr) {
    std::map<std::string, std::vector<std::string>> result;

    std::vector<std::string> tasks = ns->getRegisteredTasks();
    for (const std::string &tname : tasks) {
        result[tname] = getTaskInputPortsByType(tname, typestr);
    }
    return result;
}

std::vector<std::string> RemoteTasks::getTaskOutputPortsByType(const std::string &taskname, const std::string &typestr) {
    std::vector<std::string> result;
    RTT::TaskContext* tc = ns->getTaskContext(taskname);
    if (tc) {
        for (auto& portname : tc->ports()->getPortNames()) {
            RTT::base::OutputPortInterface* portInterfacePtr = dynamic_cast<RTT::base::OutputPortInterface*>(tc->getPort(portname));
            if (portInterfacePtr) {
                std::string type = portInterfacePtr->getTypeInfo()->getTypeName();
                if (type == typestr) {
                    result.push_back(portname);
                }
            }
        }
    }
    return result;
}

std::vector<std::string> RemoteTasks::getTaskInputPortsByType(const std::string &taskname, const std::string &typestr) {
    std::vector<std::string> result;
    RTT::TaskContext* tc = ns->getTaskContext(taskname);
    if (tc) {
        for (auto& portname : tc->ports()->getPortNames()) {
            RTT::base::InputPortInterface* inPortInterfacePtr = dynamic_cast<RTT::base::InputPortInterface*>(tc->getPort(portname));
            if (inPortInterfacePtr) {
                std::string type = inPortInterfacePtr->getTypeInfo()->getTypeName();
                if (type == typestr) {
                    result.push_back(portname);
                }
            }
        }
    }
    return result;
}
