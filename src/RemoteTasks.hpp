#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>

#include <orocos_cpp/NameService.hpp>

namespace RTT {
    class TaskContext;
    namespace base{
        class OutputPortInterface;
        class InputPortInterface;
    }
}

class RemoteTasks {

 public:

    RemoteTasks();
    virtual ~RemoteTasks(){}

    bool connect();

    std::vector<std::string> getTasks();

    RTT::TaskContext* getTaskContext(const std::string &taskname);

    std::map<std::string, std::vector<std::string>> getOutputPortsByType(const std::string &typestr);
    std::map<std::string, std::vector<std::string>> getInputPortsByType(const std::string &typestr);

    std::vector<std::string> getTaskOutputPortsByType(const std::string &taskname, const std::string &typestr);
    std::vector<std::string> getTaskInputPortsByType(const std::string &taskname, const std::string &typestr);

    RTT::base::OutputPortInterface* getOutputPort(const std::string &taskname, const std::string &portname);
    RTT::base::InputPortInterface* getInputPort(const std::string &taskname, const std::string &portname);

 private:
    std::unique_ptr<orocos_cpp::NameService> ns;

};