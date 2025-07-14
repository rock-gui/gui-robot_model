#include <orocos_cpp/orocos_cpp.hpp>
#include <orocos_cpp_base/OrocosHelpers.hpp>
#include <orocos_cpp/NameService.hpp>
#include <orocos_cpp/CorbaNameService.hpp>


#include <vizkit3d/Vizkit3DWidget.hpp>
#include "../viz/RobotVisualization.hpp"
#include <control_ui/ControlUi.h>
#include <QApplication>
#include <QDockWidget>
#include <QObject>
#include <QTimer>
#include <base/samples/Joints.hpp>

#include <boost/program_options.hpp>
#include <rtt/typelib/TypelibMarshallerBase.hpp>
#include <rtt/OutputPort.hpp>

int main(int argc, char *argv[])
{

    std::string file = "";
    std::string joint_state_port = "";
    std::string joint_state_task = "";
    std::string joint_control_port = "";
    std::string joint_control_task = "";

    boost::program_options::options_description options_desc("Usage: rock-roboviz_bin [options] <urdf/sdf file>");
    boost::program_options::positional_options_description positional_desc;
    options_desc.add_options()
        ("help,h", "print help")
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
        std::cout << "specify urdf file: rock-roboviz_bin <urdf file>" << std::endl;
        return 0;
    }

    
    std::unique_ptr<orocos_cpp::NameService> ns;
    std::shared_ptr<orocos_cpp::OrocosCpp> orocos;
    orocos_cpp::OrocosCppConfig config;
    RTT::TaskContext *localtask;

    ns = std::make_unique<orocos_cpp::CorbaNameService>();

    config.load_all_packages = true;
    config.load_typekits = true; //TODO: only load base-types 
    orocos = std::make_shared<orocos_cpp::OrocosCpp>();
    orocos->initialize(config, false);
    OrocosHelpers::initClientTask("rock-roboviz");
    localtask = OrocosHelpers::getClientTask();

    
    if (!ns->connect()) {
        std::cout << "Could not connect to Nameserver " << std::endl;
        return 0;
    }

    // get all running tasks
    std::vector<std::string> tasks = ns->getRegisteredTasks();

    // collect task/port list of tasks with /base/samples/Joints output port
    std::map<std::string, std::vector<std::string>> jointSampleTasksAndPorts;
    std::map<std::string, std::vector<std::string>> jointControlTasksAndPorts;
    for (const std::string &tname : tasks) {

        if (joint_state_task == "" || joint_state_task == tname) {

            // std::cout << "Task " << tname << std::endl;
            // RTT::corba::TaskContextProxy* tcp = orocos->getTaskContext(tname);
            RTT::TaskContext* tc = ns->getTaskContext(tname);

            if (tc) {
                for (auto& portname : tc->ports()->getPortNames()) {
                    RTT::base::OutputPortInterface* portInterfacePtr = dynamic_cast<RTT::base::OutputPortInterface*>(tc->getPort(portname));
                    if (portInterfacePtr) {
                        std::string type = portInterfacePtr->getTypeInfo()->getTypeName();
                        if (type == "/base/samples/Joints") {
                            jointSampleTasksAndPorts[tname].push_back(portname);
                            // printf("%s:%i %s %s %s\n", __PRETTY_FUNCTION__, __LINE__, tname.c_str(), portname.c_str(), type.c_str());
                        }
                    }
                    RTT::base::InputPortInterface* inPortInterfacePtr = dynamic_cast<RTT::base::InputPortInterface*>(tc->getPort(portname));
                    if (inPortInterfacePtr) {
                        std::string type = inPortInterfacePtr->getTypeInfo()->getTypeName();
                        if (type == "/base/samples/Joints") {
                            jointControlTasksAndPorts[tname].push_back(portname);
                            printf("%s:%i %s %s %s\n", __PRETTY_FUNCTION__, __LINE__, tname.c_str(), portname.c_str(), type.c_str());
                        }
                    }
                }
            }
        }
    }

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

    if (jointControlTasksAndPorts.size() >= 1) {
        printf("\nJoint control ports found, explicitly provide task and port to connect\n\n");
        for (const auto& task : jointControlTasksAndPorts) {
            for (const auto& port : task.second){
                printf("-c %s -p %s\n", task.first.c_str(), port.c_str());
            }
        }
        printf("\n");
    }

    //connect samples 
    std::string taskname = jointSampleTasksAndPorts.begin()->first;
    std::string portname = jointSampleTasksAndPorts.begin()->second.front();

    printf("connecting %s:%s\n", taskname.c_str(), portname.c_str());
    
    RTT::base::InputPortInterface *sample_port = nullptr;

    // RTT::corba::TaskContextProxy* tcp = orocos->getTaskContext(taskname);
    RTT::TaskContext* tc = ns->getTaskContext(taskname);
    RTT::base::OutputPortInterface* portInterfacePtr = dynamic_cast<RTT::base::OutputPortInterface*>(tc->getPort(portname));
    if (portInterfacePtr) {

        /* Create port */
        sample_port = portInterfacePtr->getTypeInfo()->inputPort("samples");
        if (!sample_port) {
            RTT::log(RTT::Error) << "An error occurred during port generation." << RTT::endlog();
                exit(1);
        }
        localtask->ports()->addEventPort(sample_port->getName(), *(sample_port) );

        portInterfacePtr->connectTo(sample_port);

    }

    //connect control 
    // RTT::base::OutputPortInterface *control_port = nullptr;
    RTT::OutputPort<base::samples::Joints> control_port("control");
    if (joint_control_port != "" && joint_control_task != "") {
        printf("connecting control %s:%s\n", joint_control_task.c_str(), joint_control_port.c_str());
        
        // RTT::corba::TaskContextProxy* tcp = orocos->getTaskContext(taskname);
        RTT::TaskContext* tc = ns->getTaskContext(joint_control_task);

        RTT::base::InputPortInterface* controlPortInterfacePtr = dynamic_cast<RTT::base::InputPortInterface*>(tc->getPort(joint_control_port));
        if (controlPortInterfacePtr) {

            /* Create port */
            // control_port = portInterfacePtr->getTypeInfo()->outputPort("control");
            // if (!control_port) {
            //     RTT::log(RTT::Error) << "An error occurred during port generation." << RTT::endlog();
            //         exit(1);
            // }
            localtask->ports()->addPort(control_port.getName(), control_port );

            control_port.connectTo(controlPortInterfacePtr);

        }
    }


    localtask->start();

    const RTT::types::TypeInfo* typeinfo = portInterfacePtr->getTypeInfo();
    orogen_transports::TypelibMarshallerBase * transport = dynamic_cast<orogen_transports::TypelibMarshallerBase *>(typeinfo->getProtocol(orogen_transports::TYPELIB_MARSHALLER_ID));
    orogen_transports::TypelibMarshallerBase::Handle* transportHandle = transport->createSample();
    // const Typelib::Registry& registry = transport->getRegistry();
    // const Typelib::Type *typeptr = registry.get(transport->getMarshallingType());
    RTT::base::DataSourceBase::shared_ptr datasource = transport->getDataSource(transportHandle);
    
    base::samples::Joints data;


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
    ControlUi *controlWidget = new ControlUi();
    controlWidget->initFromURDF(filepath);

    QDockWidget *controlDockWidget = new QDockWidget();
    controlDockWidget->setWidget(controlWidget);

    vizkitWidget->addDockWidget(Qt::BottomDockWidgetArea, controlDockWidget);

    QObject::connect(controlWidget, SIGNAL(newVal(base::samples::Joints)),vizPlugin,SLOT(setJointsState(base::samples::Joints)));

    vizkitWidget->show();


    // create and connect a timer that updates the data, so
    // qapplication::exec() can be used instaed of custom loop with qapplication::processevents
    QTimer updateTimer(vizkitWidget);
    QObject::connect(&updateTimer, &QTimer::timeout, [&](){
        while (sample_port->read(datasource) == RTT::NewData) {
            // create a persistent copy
            data = *reinterpret_cast<base::samples::Joints*>(datasource->getRawPointer());
        }
        // update viz only with latest value
        vizPlugin->updateData(data);
    });
    updateTimer.start(0); // runevery loop in a.exec()

    if (control_port.connected()) {
        QObject::connect(controlWidget, &ControlUi::sendSignal, [&]() {
            control_port.write(controlWidget->getJoints());
        });
    }

    a.exec();

    //cleanup
    localtask->stop();


    return 0;

}

