#include <iostream>
#include "RobotVisualization.hpp"
#include <QMessageBox>
#include <base/Logging.hpp>
#include <osg/Geode>

using namespace vizkit;
using namespace std;

struct RobotVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::Joints data;
};


RobotVisualization::RobotVisualization()
    : p(new Data)
{
}

RobotVisualization::~RobotVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> RobotVisualization::createMainNode()
{
    if(root_)
        return root_;
    else{
        loadEmptyScene();
        return root_;
    }
}

void RobotVisualization::updateMainNode ( osg::Node* node )
{
    //osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void RobotVisualization::updateDataIntern(base::samples::Joints const& value)
{
    vector<string> names = getJointNames();
    if (value.hasNames())
        names = value.names;
    else if (names.size() != value.states.size())
        throw std::runtime_error("RobotVisualization::updateDataIntern: state vector size and expected joint size differ, and there are no names in the Joints sample");

    for(uint i=0; i<names.size(); i++){
        setJointState(names[i], value.states[i].position);
    }
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(RobotVisualization)

