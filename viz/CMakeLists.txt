rock_find_qt5(REQUIRED Gui Core Widgets)
rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp OSGSegment.cpp VBOVisitor.hpp
    DEPS_PKGCONFIG kdl_parser base-types base-viz base-lib base-logging sdformat urdfdom
    MOC RobotVisualization.hpp
    HEADERS RobotVisualization.hpp RobotModel.h OSGSegment.h VBOVisitor.hpp)
