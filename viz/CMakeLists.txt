include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src/urdf)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src/urdf/urdfdom_headers)
rock_vizkit_plugin(robot_model-viz
    RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp
    DEPS_PKGCONFIG kdl_parser base-types
    MOC RobotVisualization.hpp RobotModel.h OSGHelpers.hpp
    HEADERS RobotVisualization.hpp RobotModel.h)

