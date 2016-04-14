#include <iostream>
#include "RobotVisualization.hpp"
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
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
    this->modelPos = new osg::PositionAttitudeTransform();
    connect(this, SIGNAL(propertyChanged(QString)), this, SLOT(handlePropertyChanged(QString)));
    setJointsSize(0.1);
    setFramesEnabled(false);
    setSegmentNamesEnabled(false);
    setFollowModelWithCamera(false);
}

RobotVisualization::~RobotVisualization()
{
    delete p;
    deleteFrameVisualizers();
}

void RobotVisualization::handlePropertyChanged(QString property){
    if(property == "frame"){
        std::vector<std::string>::iterator it;
        it = std::find(segmentNames_.begin(), segmentNames_.end(), getVisualizationFrame().toStdString());
        if(it != segmentNames_.end()){
            setRootLink(getVisualizationFrame());
        }
        else{
            if(getVisualizationFrame() != "world_osg"){
                QMessageBox::information(NULL, "vizkit3d::RobotVisualization", "The visualization frame was changed to "+getVisualizationFrame()+", but "\
                                         "this is not a known body part. Make sure to set 'rootFrame' to a reasonable body part name.");
            }
            else{ //Its world_osg, set to original root if it was determined yet
                if(original_root_name_ != "")
                    setRootLink(original_root_name_.c_str());
            }
        }
    }
}

void RobotVisualization::highlightSegment(QString link_name){
    bool highlighted = toggleHighlight(link_name.toStdString());
    if(!highlighted)
        toggleHighlight(link_name.toStdString());
}

void RobotVisualization::deHighlightSegment(QString link_name){
    bool highlighted = toggleHighlight(link_name.toStdString());
    if(highlighted)
        toggleHighlight(link_name.toStdString());
}

void RobotVisualization::showSegmentText(QString link_name, QString text){
    OSGSegment* seg = getSegment(link_name.toStdString());
    seg->attachTextLabel(text.toStdString());
}

void RobotVisualization::hideSegmentText(QString link_name){
    OSGSegment* seg = getSegment(link_name.toStdString());
    seg->removeTextLabel();
}

void RobotVisualization::setModelFile(QString modelFile)
{
    //Extact file extension
    QStringList tokens = modelFile.split(QChar('.'));
    QString ext = tokens.back();

    LOG_INFO("loading %s", modelFile.toLatin1().data());
    bool st;
    st = load(modelFile);
    if(!st)
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
        assert(segment);
        vizkit3d::RigidBodyStateVisualization* frame =
                new vizkit3d::RigidBodyStateVisualization(this);
        frame->setPluginName(QString::fromStdString(segments[i]));

        osg::ref_ptr<osg::Group> frame_attachment = frame->getRootNode();
        osg::ref_ptr<osg::StateSet> state = frame_attachment->getOrCreateStateSet();
        state->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

        segment->getGroup()->addChild(frame_attachment);
        _frameVisualizers[segments[i]] = frame;
    }
    setFramesEnabled(areFramesEnabled());
    setSegmentNamesEnabled(areSegmentNamesEnabled());

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
    for (size_t i=0; i<segmentNames_.size(); i++){
        setFrameEnabled(QString(segmentNames_[i].c_str()), framesEnabled_, joints_size);
    }
}

void RobotVisualization::setFrameEnabled(QString segment_name, bool value, double size){
    map<string, RigidBodyStateVisualization*>::iterator it;
    it=_frameVisualizers.find(segment_name.toStdString());

    if(size<0){
        size = joints_size;
    }

    if(it == _frameVisualizers.end()){
        std::clog << "Tried to enable frame display of link " << segment_name.toStdString() << ", but could not find it." << std::endl;
        return;
    }
    if(value){
        it->second->setSize(size);
        it->second->setPluginEnabled(true);
    }
    else{
        it->second->setSize(1e-10);
        it->second->setPluginEnabled(false);
    }
}

bool RobotVisualization::areSegmentNamesEnabled() const
{
    return segmentNamesEnabled_;
}

void RobotVisualization::setSegmentNamesEnabled(bool value)
{
    segmentNamesEnabled_ = value;
    for (size_t i=0; i<segmentNames_.size(); i++){
        OSGSegment* seg = getSegment(segmentNames_[i]);
        if(value)
            seg->attachTextLabel();
        else
            seg->removeTextLabel();
    }
}

QQuaternion RobotVisualization::getRotation(QString source_frame, QString target_frame){
    osg::Matrixd tr = getRelativeTransform(target_frame.toStdString(), source_frame.toStdString());
    osg::Quat q = tr.getRotate();
    return QQuaternion(q.w(), q.x(), q.y(), q.z());
}

QVector3D RobotVisualization::getTranslation(QString source_frame, QString target_frame){
    osg::Matrixd tr = getRelativeTransform(target_frame.toStdString(), source_frame.toStdString());
    osg::Vec3d t = tr.getTrans();
    return QVector3D(t.x(),t.y(),t.z());
}

bool RobotVisualization::getFollowModelWithCamera() const{
	return followModelWithCamera;
}
void RobotVisualization::setFollowModelWithCamera(bool value){
	followModelWithCamera = value;
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
    // Update the main node using the data in p->pos
    if (p->pos.hasValidPosition()){
		osg::Vec3d position(p->pos.position.x(), p->pos.position.y(), p->pos.position.z());
		modelPos->setPosition(position);

		if (followModelWithCamera){
			Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget *>(this->parent());

			QVector3D lookAtPos, eyePos, upVector;
			widget->getCameraView(lookAtPos, eyePos, upVector);
			widget->setCameraEye(eyePos.x(),eyePos.y(),eyePos.z());
			widget->setCameraLookAt(p->pos.position.x(), p->pos.position.y(), p->pos.position.z());
			widget->setCameraUp(upVector.x(),upVector.y(),upVector.z());
		}
    }

    if (p->pos.hasValidOrientation()){
		osg::Quat orientation(p->pos.orientation.x(), p->pos.orientation.y(), p->pos.orientation.z(),p->pos.orientation.w());
		modelPos->setAttitude(orientation);
    }

}


double RobotVisualization::getJointsSize() const
{
    return joints_size;
}


void RobotVisualization::setJointsSize(double size)
{
    joints_size = size;
    setFramesEnabled(framesEnabled_);
}

void RobotVisualization::setJointsState(const base::samples::Joints &sample){
    vector<string> names = getJointNames();
    if (sample.hasNames())
        names = sample.names;
    if (names.size() != sample.elements.size())
        throw std::runtime_error("RobotVisualization::updateDataIntern: state vector size and expected joint size differ, and there are no names in the Joints sample");

    for(uint i=0; i<names.size(); i++){
        if(base::isUnknown(sample[i].position) || base::isInfinity(sample[i].position)){
            if(names.size()){
                LOG_ERROR("Position of joint %s is invalid: %d", sample.names[i].c_str(), sample[i].position);
            }
            else{
                LOG_ERROR("Position of joint %d is invalid: %d", i, sample[i].position);
            }
            //throw std::runtime_error("RobotVisualization::updateDataIntern: invalid joint position detected.");
            setJointState(names[i], 0);
        }
        else{
            setJointState(names[i], sample[i].position);
        }
    }
}

void RobotVisualization::updateDataIntern(base::samples::Joints const& sample)
{
    setJointsState(sample);
}

void RobotVisualization::updateDataIntern(base::samples::RigidBodyState const& pos){
	p->pos = pos;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(RobotVisualization)

