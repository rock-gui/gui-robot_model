

#include "RemoteTasks.hpp"
#include "LocalTask.hpp"

#include <vizkit3d/Vizkit3DWidget.hpp>
#include "../viz/RobotVisualization.hpp"
#include <control_ui/ControlUi.h>
#include <QApplication>
#include <QDockWidget>
#include <QObject>
#include <QTimer>

#include <boost/program_options.hpp>


int main(int argc, char *argv[])
{

    std::string file = "";
    std::string joint_state_port = "";
    std::string joint_state_task = "";
    std::string joint_control_port = "";
    std::string joint_control_task = "";
    std::string host = "";

    boost::program_options::options_description options_desc("Usage: rock-roboviz_bin [options] <urdf/sdf file>");
    boost::program_options::positional_options_description positional_desc;
    options_desc.add_options()
        ("help,h", "print help")
        ("host", boost::program_options::value(&host), "Corba host to connect to")
        ("file,f", boost::program_options::value(&file), "/path/to/model/file, can also be provided as last paramater")
        ("joint_state_port,s", boost::program_options::value(&joint_state_port), "Use joint state port to read data from")
        ("joint_state_task,t", boost::program_options::value(&joint_state_task), "Use joint state port to read data from")
        ("joint_control_port,p", boost::program_options::value(&joint_control_port), "Use joint control port to write data to")
        ("joint_control_task,c", boost::program_options::value(&joint_control_task), "Use joint control port to write data to")
        ;

    
    positional_desc.add("file", -1);
    

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(options_desc).positional(positional_desc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << options_desc << std::endl;
        return 0;
    }


    if (file == "") {
        std::cout << "specify urdf file: rock-roboviz_bin [options] <urdf file>" << std::endl;
        return 0;
    }

    RTT::Logger::log().setLogLevel(RTT::Logger::Never);    

    LocalTask localtask("rock-roboviz");

    // collect remote task data
    RemoteTasks taskinfo;
    if (!taskinfo.connect()) {
        std::cout << "Could not connect to Nameserver " << std::endl;
        return 0;
    }

    // collect task/port list of tasks with /base/samples/Joints output port
    if (joint_state_task == "" && joint_state_port == "") {
        std::map<std::string, std::vector<std::string>> jointSampleTasksAndPorts = taskinfo.getOutputPortsByType("/base/samples/Joints");
    
        if (jointSampleTasksAndPorts.size() == 0) {
            printf("No joint state producer port found in running tasks\n");
            exit(1);
        }
        if (jointSampleTasksAndPorts.size() > 1) {
            printf("Multiple joint state producer tasks. Don't know which to use, explicitly provide one\n");
            for (const auto& task : jointSampleTasksAndPorts) {
                printf("-t %s\n", task.first.c_str());
            }
            exit(1);
        }

        if (jointSampleTasksAndPorts.begin()->second.size() > 1) {
            printf("Multiple joint state producer ports. Don't know which to use, explicitly provide one\n");
            for (const auto& task : jointSampleTasksAndPorts) {
                for (const auto& port : task.second){
                    printf("-t %s -s %s\n", task.first.c_str(), port.c_str());
                }
            }
            exit(1);
        }
        // only found single port, apply automatically
        joint_state_task = jointSampleTasksAndPorts.begin()->first;
        joint_state_port = jointSampleTasksAndPorts.begin()->second.front();
    }

    // task selected, port not
    if (joint_state_port == ""){
        std::vector<std::string> ports = taskinfo.getTaskOutputPortsByType(joint_state_task, "/base/samples/Joints"); 

        if (ports.size() == 0) {
            printf("No joint state producer ports on task provided.\n");
            exit(1);
        }
        if (ports.size() > 1) {
            printf("Multiple joint state producer ports on task. Don't know which to use, explicitly provide one\n");
            for (const auto& port : ports){
                printf("-s %s\n", port.c_str());
                exit(1);
            }
        }
        joint_state_port = ports.front();
    }

    if (joint_control_task == "" && joint_control_port == "") {
        std::map<std::string, std::vector<std::string>> jointControlTasksAndPorts = taskinfo.getInputPortsByType("/base/commands/Joints");
        if (jointControlTasksAndPorts.size() >= 1) {
            printf("\nJoint control ports found, explicitly provide task and port to connect\n\n");
            for (const auto& task : jointControlTasksAndPorts) {
                for (const auto& port : task.second){
                    printf("-c %s -p %s\n", task.first.c_str(), port.c_str());
                }
            }
            printf("\n");
        }
    }


    printf("connecting %s:%s\n", joint_state_task.c_str(), joint_state_port.c_str());


    //connect samples
    RTT::base::OutputPortInterface* portInterfacePtr = taskinfo.getOutputPort(joint_state_task, joint_state_port);
    if (portInterfacePtr) {
        localtask.connectSamples(portInterfacePtr);
    }

    // connect control 
    if (joint_control_port != "" && joint_control_task != "") {
        printf("connecting control %s:%s\n", joint_control_task.c_str(), joint_control_port.c_str());
        RTT::base::InputPortInterface* controlPortInterfacePtr = taskinfo.getInputPort(joint_control_task, joint_control_port);
        if (controlPortInterfacePtr) {
            localtask.connectCommands(controlPortInterfacePtr);
        }
    }
    
    // create GUI

    QString filepath(file.c_str());
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
    ControlUi *controlWidget = nullptr;
    if (joint_control_task != "" && joint_control_port != "") {
        ControlUi *controlWidget = new ControlUi();
        controlWidget->initFromURDF(filepath);

        QDockWidget *controlDockWidget = new QDockWidget();
        controlDockWidget->setWidget(controlWidget);

        vizkitWidget->addDockWidget(Qt::BottomDockWidgetArea, controlDockWidget);

        QObject::connect(controlWidget, SIGNAL(newVal(base::samples::Joints)),vizPlugin,SLOT(setJointsState(base::samples::Joints)));

        QObject::connect(controlWidget, &ControlUi::sendSignal, [&]() {
            localtask.sendCommand(controlWidget->getJoints());
        });
    }

    vizkitWidget->show();


    // create and connect a timer that updates the data, so
    // qapplication::exec() can be used instaed of custom loop with qapplication::processevents
    QTimer updateTimer(vizkitWidget);
    QObject::connect(&updateTimer, &QTimer::timeout, [&](){
        base::samples::Joints data;
        if (localtask.getSample(&data)) {
            vizPlugin->updateData(data);
        }
    });
    updateTimer.start(20); // update every 20ms




    return a.exec();
}

