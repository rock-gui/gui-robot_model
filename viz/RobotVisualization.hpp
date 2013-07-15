#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/samples/Joints.hpp>
#include "RobotModel.h"


namespace vizkit
{
class RobotVisualization
        : public vizkit::Vizkit3DPlugin<base::samples::Joints>, public RobotModel
        , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(QString modelFile READ modelFile WRITE setModelFile NOTIFY modelFileChanged)
public:
    RobotVisualization();
    ~RobotVisualization();

    void setModelFile(QString modelFile){
        //Extact file extension
        QStringList tokens = modelFile.split(QChar('.'));
        QString ext = tokens.back();

        LOG_INFO("loading %s", modelFile.toLatin1().data());
        if(!load(modelFile))
        {
            QMessageBox::critical(NULL, "vizkit::RobotVisualization", "cannot load " + modelFile + ", it either does not exist or is not a proper robot model file");
        }
        else
        {
            _modelFile = modelFile;
        }
        //        emit(modelFileChanged(_modelFile));
    }

    QString modelFile(){
        return _modelFile;
    }

    Q_INVOKABLE void updateData(base::samples::Joints const &sample)
    {vizkit::Vizkit3DPlugin<base::samples::Joints>::updateData(sample);}

signals:
    void modelFileChanged(QString);

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::Joints const& plan);

private:
    struct Data;
    Data* p;
    QString _modelFile;
};
}
#endif
