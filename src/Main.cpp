#include "vizkit3d/Vizkit3DWidget.hpp"
#include <vizkit3d/RobotVisualization.hpp>
#include <QApplication>

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cout << "specify urdf file: rock-roboviz <urdf file>" << std::endl;
        return 0;
    }

    std::cout << "loading file: " << argv[1] << std::endl;
    std::string filepath(argv[1]);

    QApplication a(argc, argv);
    vizkit3d::Vizkit3DWidget window;

    // just for debug purposes
    QStringList* plugins = window.getAvailablePlugins();
    for ( const auto& i : *plugins  )
    {
        std::cout << i.toStdString() << std::endl;
    }

    // load RobotVisualization plugin
    QObject* robot_viz = window.loadPlugin("robot_model","");
    vizkit3d::RobotVisualization* vizPlugin = dynamic_cast<vizkit3d::RobotVisualization *>(robot_viz);

    // load the file
    vizPlugin->loadFromFile(QString(filepath.c_str()));

    window.show();

    return a.exec();
}