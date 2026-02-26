
#pragma once

#include <QObject>

class LocalTask;
class ControlUi;
namespace vizkit3d {
    class RobotVisualization;
}

class Adapter : public QObject {
    Q_OBJECT
public:
    Adapter(LocalTask *localtask,
            vizkit3d::RobotVisualization* vizPlugin,
            QObject *parent = nullptr);

    void setControlWidget(ControlUi *controlWidget);

public slots:
    void controlSend();
    void timer();

private:
    LocalTask *localtask;
    vizkit3d::RobotVisualization* vizPlugin;
    ControlUi *controlWidget;
};
