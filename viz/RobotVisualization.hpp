#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/Joints.hpp>
#include "RobotModel.h"

namespace vizkit3d
{
    class RigidBodyStateVisualization;

class RobotVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::Joints>, public RobotModel
        , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(QString modelFile READ modelFile WRITE setModelFile)
    Q_PROPERTY(bool framesEnabled READ areFramesEnabled WRITE setFramesEnabled)

public:
    RobotVisualization();
    ~RobotVisualization();

    void setModelFile(QString modelFile);
    QString modelFile() const;

    Q_INVOKABLE void updateData(base::samples::Joints const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

public slots:
    bool areFramesEnabled() const;
    void setFramesEnabled(bool value);

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::Joints const& plan);

    void deleteFrameVisualizers();

private:
    struct Data;
    Data* p;
    QString _modelFile;
    bool framesEnabled_;
    std::map<std::string, RigidBodyStateVisualization*> _frameVisualizers;
};
}
#endif
