rock_find_qt4()

# Add further possible sdformat options here if necessary
pkg_search_module(SDFORMAT REQUIRED sdformat sdformat9 sdformat12 sdformat14 sdformat15)

rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp OSGSegment.cpp VBOVisitor.hpp
    DEPS_PKGCONFIG kdl_parser base-types base-viz base-lib base-logging ${SDFORMAT_MODULE_NAME} urdfdom
    MOC RobotVisualization.hpp
    HEADERS RobotVisualization.hpp RobotModel.h OSGSegment.h VBOVisitor.hpp)
