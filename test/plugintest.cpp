#include <QtCore>
#include <iostream>
#include <QApplication>
#include "vizkit3d/RobotVisualization.hpp"
#include "vizkit3d/RobotModel.h"
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "my_timer.h"

vizkit3d::RobotVisualization* plugin;




int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    vizkit3d::Vizkit3DWidget *widget = new vizkit3d::Vizkit3DWidget();
    widget->show();

    plugin = new vizkit3d::RobotVisualization();
    plugin->setModelFile("../../test_data/spacebot_arm/spacebot_arm.urdf");
    widget->addPlugin(plugin);

    base::samples::Joints joints;
    joints.resize(2);
    joints.names[0] = "joint_1";
    joints.names[1] = "joint_2";
    joints.elements[0].position = 2.1;
    joints.elements[1].position = 2.1;
    plugin->updateData(joints);

    MyTimer t;
    t.singleShot(500, &t, SLOT(update_transform()));

    base::samples::RigidBodyState rbs;
    rbs.position(0) = 1;
    rbs.position(1) = 0.5;
    rbs.position(2) = 0;
    plugin->updateRBS(rbs);

    app.exec();

    return 0;
}
