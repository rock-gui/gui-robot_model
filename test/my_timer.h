#include <QTimer>
#include "vizkit3d/RobotVisualization.hpp"
#include <osg/io_utils>

extern vizkit3d::RobotVisualization* plugin;

class MyTimer :public QTimer{
    Q_OBJECT
public:
    MyTimer(){}
    ~MyTimer(){}

public slots:
    void update_transform(){
        std::string source_frame = "base_link";
        std::string target_frame = "link_6";

        osg::Matrixd t=plugin->getRelativeTransform(source_frame, target_frame);
        std::stringstream ss;
        ss << source_frame << " -> " << target_frame << " :\nT: " << t.getTrans()<<"\n";
        ss << "R: "<<t.getRotate()<<"\n";

        std::cout << ss.str() << std::endl;
        plugin->getSegment(source_frame)->attachTextLabel(ss.str());
        plugin->setFrameEnabled(source_frame.c_str(), true, 0.05);
        plugin->setFrameEnabled(target_frame.c_str(), true, 0.05);
    }
};
