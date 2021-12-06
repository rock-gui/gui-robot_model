#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <QMessageBox>
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
    Q_PROPERTY(bool followModelWithCamera_ READ getFollowModelWithCamera WRITE setFollowModelWithCamera)
    Q_PROPERTY(QString rootLink READ getRootLink WRITE setRootLink)
    Q_PROPERTY(bool segmentNamesEnabled READ areSegmentNamesEnabled WRITE setSegmentNamesEnabled)
    Q_PROPERTY(bool showVisuals READ areVisualsEnabled WRITE setVisualsEnabled)
    Q_PROPERTY(bool showCollision READ areCollisionsEnabled WRITE setCollisionsEnabled)
    Q_PROPERTY(bool showInertias READ areInertiasEnabled WRITE setInertiasEnabled)

public:
    RobotVisualization();
    ~RobotVisualization();

    void setModelFile(QString modelFile);
    QString modelFile() const;

    Q_INVOKABLE void clearMeshCache();

    Q_INVOKABLE void updateData(base::samples::Joints const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

    Q_INVOKABLE void updateData(base::samples::RigidBodyState const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}
    Q_INVOKABLE void updateRBS(base::samples::RigidBodyState const &sample)
    {updateData(sample);}

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

    /** Loads a model from a file
     *
     * @param path the file path
     * @param type the file type, as either "auto", "urdf" and "sdf". If given
     *   "auto", the file type will be guessed based on the file extension
     */
    void loadFromFile(QString path, QString type = "auto");

    /** Loads a model from the XML string instead of from a file
     *
     * @param xml the XML document
     * @param type the document type, as either "urdf" and "sdf"
     * @param rootPrefix the path relative to which ressources (such as meshes)
     *   should be resolved
     */
    void loadFromString(QString value, QString type, QString rootPrefix = "");

    bool areFramesEnabled() const;
    void setFramesEnabled(bool value);
    //If negative numker is given for size, (global) joint_size is taken.
    void setFrameEnabled(QString segment_name, bool value, double size=-1);
    /** Joints Frame using RigidBodyStateVisualization
    * The default is 0.1
    */
    double getJointsSize() const;
    /** Joints Frame using RigidBodyStateVisualization
    */
    void setJointsSize(double size);
    void highlightSegment(QString);
    void deHighlightSegment(QString);
    //If no text is given, the segment name in written
    void showSegmentText(QString link_name, QString text="");
    void hideSegmentText(QString link_name);
    bool areSegmentNamesEnabled() const;
    void setSegmentNamesEnabled(bool value);
    bool areVisualsEnabled() const;
    void setVisualsEnabled(bool value);
    bool areCollisionsEnabled() const;
    void setCollisionsEnabled(bool value);
    bool areInertiasEnabled() const;
    void setInertiasEnabled(bool value);
    QQuaternion getRotation(QString source_frame, QString target_frame);
    QVector3D getTranslation(QString source_frame, QString target_frame);

    /**
     * camera rotation center follows the robot
     */
    bool getFollowModelWithCamera() const;
    void setFollowModelWithCamera(bool value);

    void handlePropertyChanged(QString);
    void setJointsState(base::samples::Joints const& sample);


protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::Joints const& sample);
    virtual void updateDataIntern(base::samples::RigidBodyState const& pos);

    void createFrameVisualizers();
    void deleteFrameVisualizers();

private:
    struct Data;
    bool framesEnabled_ = false;
    bool followModelWithCamera_;
    bool segmentNamesEnabled_ = false;
    bool visualsEnabled_ = true;
    bool collisionsEnabled_ = false;
    bool inertiasEnabled_ = false;
    double joints_size;
    Data* p;
    QString _modelFile;
    std::map<std::string, RigidBodyStateVisualization*> _frameVisualizers;

    osg::ref_ptr<osg::PositionAttitudeTransform> modelPos;
};
}
#endif
