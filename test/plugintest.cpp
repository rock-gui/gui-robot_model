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
    plugin->setModelFile("/media/data/Development/besman/bundles/besman_test/data/aila.urdf");
    widget->addPlugin(plugin);

    base::samples::Joints joints;
    joints.resize(2);
    joints.names[0] = "J_Knees";
    joints.names[1] = "J_Hip";
    joints.elements[0].position = 2.1;
    joints.elements[1].position = 2.1;
    plugin->updateData(joints);

    MyTimer t;
    t.singleShot(500, &t, SLOT(update_transform()));

    app.exec();

    return 0;
}
