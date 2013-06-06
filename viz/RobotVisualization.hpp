#ifndef robot_model_RobotVisualization_H
#define robot_model_RobotVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/Joints.hpp>
#include <base/Logging.hpp>
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

        //Check if urdf or collada and load it
        std::cout << QString::compare(ext, "urdf", Qt::CaseInsensitive) << "   " << ext.toLatin1().data() <<std::endl;
        if(QString::compare(ext, "urdf", Qt::CaseInsensitive) == 0){
            LOG_INFO("%s is a URDF model file", modelFile.toLatin1().data());
            _modelFile = modelFile;
            if(!loadURDF(_modelFile))
                throw std::runtime_error("Error while loading model file");
        }
        else if(QString::compare(ext, "xml", Qt::CaseInsensitive) == 0){
            LOG_WARN("Could not determine model format from %s. Assuming it is a URDF model file",
                     modelFile.toLatin1().data());
            _modelFile = modelFile;
            if(!loadURDF(_modelFile))
                throw std::runtime_error("Error while loading model file");
        }
        else if(QString::compare(ext, "zae", Qt::CaseInsensitive) == 0
                || QString::compare(ext, "dae", Qt::CaseInsensitive) == 0){
            LOG_INFO("%s is a Collada model file", modelFile.toLatin1().data());
            _modelFile = modelFile;
            if(!loadCollada(_modelFile))
                throw std::runtime_error("Error while loading model file");
        }
        else{
            LOG_ERROR("Could not determine model format from %s. Only Collada archives (.zae), collada models (.dae) and URDF files (.urdf) are supported.",
                      modelFile.toLatin1().data());
            throw std::runtime_error("Could not determine model file format.");
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
