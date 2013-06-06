#include <iostream>
#include "RobotVisualization.hpp"

using namespace vizkit;

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
    for(uint i=0; i<value.names.size(); i++){
        setJointState(value.names[i], value.states[i].position);
    }
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(RobotVisualization)

