#include <QtCore>
#include <iostream>
#include <QApplication>
#include "vizkit3d/RobotVisualization.hpp"
#include "vizkit3d/RobotModel.h"
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    vizkit3d::Vizkit3DWidget *widget = new vizkit3d::Vizkit3DWidget();
    widget->show();

    vizkit3d::RobotVisualization* plugin = new vizkit3d::RobotVisualization();
    plugin->setModelFile("/media/data/Development/besman/bundles/besman_test/data/aila.urdf");
    widget->addPlugin(plugin);

    base::samples::Joints joints;
    joints.resize(2);
    joints.names[0] = "J_Knees";
    joints.names[1] = "J_Hip";
    joints.elements[0].position = 1.4;
    joints.elements[1].position = 1.4;
    plugin->updateData(joints);

    app.exec();

    return 0;
}
