#include "vizkit3d/Vizkit3DWidget.hpp"
#include <vizkit3d/RobotVisualization.hpp>
#include <control_ui/ControlUi.h>
#include <QApplication>
#include <QDockWidget>
#include <QObject>
#include <base/samples/Joints.hpp>

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cout << "specify urdf file: rock-roboviz <urdf file>" << std::endl;
        return 0;
    }

    QString filepath(argv[1]);
    std::cout << "Loading urdf file: " << filepath.toStdString() << std::endl;


    QApplication a(argc, argv);

    // ---- load Vizkit3D and RobotVisualization plugin
    vizkit3d::Vizkit3DWidget *vizkitWidget = new vizkit3d::Vizkit3DWidget();

    QObject* robotViz = vizkitWidget->loadPlugin("robot_model","");
    if(robotViz == nullptr)
        throw std::runtime_error("Could not load 'robot_model' plugin");

    vizkit3d::RobotVisualization* vizPlugin = dynamic_cast<vizkit3d::RobotVisualization *>(robotViz);
    if(vizPlugin == nullptr)
        throw std::runtime_error("Could not cast the plugin into RobotVisualization.");

    // load the model file
    vizPlugin->setModelFile(filepath);


    // ----- load control ui
    ControlUi *controlWidget = new ControlUi();
    controlWidget->initFromURDF(filepath);

    QDockWidget *controlDockWidget = new QDockWidget();
    controlDockWidget->setWidget(controlWidget);

    vizkitWidget->addDockWidget(Qt::BottomDockWidgetArea, controlDockWidget);

    QObject::connect(controlWidget, SIGNAL(newVal(base::samples::Joints)),vizPlugin,SLOT(setJointsState(base::samples::Joints)));

    vizkitWidget->show();

    return a.exec();
}