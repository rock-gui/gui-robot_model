
//must be first so rtt can have its signals and slots unmolested by qt.
#include "LocalTask.hpp"

#include "Adapter.hpp"
#include <control_ui/ControlUi.h>
#include "../viz/RobotVisualization.hpp"

Adapter::Adapter(LocalTask *localtask,
                 vizkit3d::RobotVisualization* vizPlugin,
                 QObject *parent)
    : QObject(parent),
      localtask(localtask),
      vizPlugin(vizPlugin),
      controlWidget(nullptr)
{
}

void Adapter::setControlWidget(ControlUi *controlWidget) {
    this->controlWidget = controlWidget;
}

void Adapter::controlSend() {
    if(controlWidget) {
        localtask->sendCommand(controlWidget->getJoints());
    }
}

void Adapter::timer() {
    base::samples::Joints data;
    if (localtask->getSample(&data)) {
        vizPlugin->updateData(data);
    }
}

