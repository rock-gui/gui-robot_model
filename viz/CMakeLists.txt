rock_find_qt4()
rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp
    DEPS_PKGCONFIG kdl_parser base-types base-viz base-lib base-logging sdformat urdfdom
    MOC RobotVisualization.hpp
    HEADERS RobotVisualization.hpp RobotModel.h)


