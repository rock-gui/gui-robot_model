#include <iostream>
#include "RobotVisualization.hpp"
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <QMessageBox>
#include <base/Logging.hpp>
#include <osg/Geode>

using namespace vizkit3d;
using namespace std;

struct RobotVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::Joints data;
    base::samples::RigidBodyState pos;
};


RobotVisualization::RobotVisualization()
    : p(new Data)
{
    this->framesEnabled_ = false;
    this->joints_size = 0.1;
    this->modelPos = new osg::PositionAttitudeTransform();
}

RobotVisualization::~RobotVisualization()
{
    delete p;
    deleteFrameVisualizers();
}

void RobotVisualization::setModelFile(QString modelFile)
{
    //Extact file extension
    QStringList tokens = modelFile.split(QChar('.'));
    QString ext = tokens.back();

    LOG_INFO("loading %s", modelFile.toLatin1().data());
    if(!load(modelFile))
        QMessageBox::critical(NULL, "vizkit3d::RobotVisualization", "cannot load " + modelFile + ", it either does not exist or is not a proper robot model file");
    else
        _modelFile = modelFile;
    //emit propertyChanged("modelFile");

    // Now create a RBS visualization for each of the frames in the model
    deleteFrameVisualizers();
    vector<string> segments = getSegmentNames();
    for (std::size_t i = 0; i != segments.size(); ++i)
    {
        OSGSegment* segment = getSegment(segments[i]);
        vizkit3d::RigidBodyStateVisualization* frame =
                new vizkit3d::RigidBodyStateVisualization(this);
        frame->setPluginName(QString::fromStdString(segments[i]));
        frame->setPluginEnabled(framesEnabled_);
        frame->resetModel(joints_size);
        segment->getGroup()->addChild(frame->getRootNode());
        _frameVisualizers[segments[i]] = frame;
    }

    emit childrenChanged();
}

void RobotVisualization::deleteFrameVisualizers()
{
    for (map<string, RigidBodyStateVisualization*>::iterator it = _frameVisualizers.begin(); it != _frameVisualizers.end(); ++it)
        delete it->second;
    _frameVisualizers.clear();
}

bool RobotVisualization::areFramesEnabled() const
{
    return framesEnabled_;
}

void RobotVisualization::setFramesEnabled(bool value)
{
    framesEnabled_ = value;
    for (map<string, RigidBodyStateVisualization*>::iterator it = _frameVisualizers.begin(); it != _frameVisualizers.end(); ++it)
        it->second->setPluginEnabled(value);
}

QString RobotVisualization::modelFile() const{
    return _modelFile;
}

osg::ref_ptr<osg::Node> RobotVisualization::createMainNode()
{



    if(!root_){
        loadEmptyScene();
    }

    modelPos->addChild(root_);

    return modelPos;

}

void RobotVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);


    // Update the main node using the data in p->pos
    osg::Vec3d position(p->pos.position.x(), p->pos.position.y(), p->pos.position.z());
    modelPos->setPosition(position);

    osg::Quat orientation(p->pos.orientation.x(), p->pos.orientation.y(), p->pos.orientation.z(),p->pos.orientation.w());
    modelPos->setAttitude(orientation);

}


double RobotVisualization::getJointsSize() const
{
    return joints_size;
}


void RobotVisualization::setJointsSize(double size)
{
    joints_size = size;
    for (map<string, RigidBodyStateVisualization*>::iterator it = _frameVisualizers.begin(); it != _frameVisualizers.end(); ++it)
        it->second->resetModel(joints_size);
}

void RobotVisualization::updateDataIntern(base::samples::Joints const& value)
{
    vector<string> names = getJointNames();
    if (value.hasNames())
        names = value.names;
    if (names.size() != value.elements.size())
        throw std::runtime_error("RobotVisualization::updateDataIntern: state vector size and expected joint size differ, and there are no names in the Joints sample");

    for(uint i=0; i<names.size(); i++){
        if(base::isUnknown(value[i].position) || base::isInfinity(value[i].position)){
            if(names.size()){
                LOG_ERROR("Position of joint %s is invalid: %d", value.names[i].c_str(), value[i].position);
            }
            else{
                LOG_ERROR("Position of joint %d is invalid: %d", i, value[i].position);
            }
            //throw std::runtime_error("RobotVisualization::updateDataIntern: invalid joint position detected.");
            setJointState(names[i], 0);
        }
        else{
            setJointState(names[i], value[i].position);
        }
    }
}

void RobotVisualization::updateDataIntern(base::samples::RigidBodyState const& pos){
	p->pos = pos;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(RobotVisualization)

