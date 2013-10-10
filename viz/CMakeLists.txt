rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp
    DEPS_PKGCONFIG kdl_parser base-types base-viz
    MOC RobotVisualization.hpp RobotModel.h OSGHelpers.hpp
    HEADERS RobotVisualization.hpp RobotModel.h)

