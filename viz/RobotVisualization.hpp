#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "RobotModel.h"

namespace vizkit3d
{
    class RigidBodyStateVisualization;

class RobotVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::Joints>,
          public vizkit3d::VizPluginAddType<base::samples::RigidBodyState>,
          public RobotModel
        , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(QString modelFile READ modelFile WRITE setModelFile)
    Q_PROPERTY(bool framesEnabled READ areFramesEnabled WRITE setFramesEnabled)
    Q_PROPERTY(double jointsSize READ getJointsSize WRITE setJointsSize)
    Q_PROPERTY(bool followModelWithCamera READ getFollowModelWithCamera WRITE setFollowModelWithCamera)
    Q_PROPERTY(QString rootLink READ getRootLink WRITE setRootLink)
    Q_PROPERTY(bool segmentNamesEnabled READ areSegmentNamesEnabled WRITE setSegmentNamesEnabled)

public:
    RobotVisualization();
    ~RobotVisualization();

    void setModelFile(QString modelFile);
    QString modelFile() const;

    Q_INVOKABLE void updateData(base::samples::Joints const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

    Q_INVOKABLE void updateData(base::samples::RigidBodyState const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

public slots:
    void setRootLink(QString segment_name){
        bool st= relocateRoot(segment_name.toStdString());
        if(!st){
            QMessageBox::critical(NULL, "vizkit3d::RobotVisualization", "Could not set root link to "+segment_name+"."\
                                 "Does this body part exist?");
        }
    }
    QString getRootLink()
    {return QString(current_root_name_.c_str());}

    bool areFramesEnabled() const;
    void setFramesEnabled(bool value);
    /** Joints Frame using RigidBodyStateVisualization
    * The default is 0.1
    */
    double getJointsSize() const;
    /** Joints Frame using RigidBodyStateVisualization
    */
    void setJointsSize(double size);
    void highlightSegment(QString);
    void showSegmentName(QString);
    void hideSegmentName(QString link_name);
    bool areSegmentNamesEnabled() const;
    void setSegmentNamesEnabled(bool value);

    /**
     * camera rotation center follows the robot
     */
    bool getFollowModelWithCamera() const;
    void setFollowModelWithCamera(bool value);

    void handlePropertyChanged(QString);


protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::Joints const& plan);
    virtual void updateDataIntern(base::samples::RigidBodyState const& pos);

    void deleteFrameVisualizers();

private:
    struct Data;
    bool framesEnabled_;
    bool followModelWithCamera;
    bool segmentNamesEnabled_;
    double joints_size;
    Data* p;
    QString _modelFile;
    std::map<std::string, RigidBodyStateVisualization*> _frameVisualizers;

    osg::ref_ptr<osg::PositionAttitudeTransform> modelPos;
};
}
#endif
