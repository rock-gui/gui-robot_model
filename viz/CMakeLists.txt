rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL Widgets)

# Add further possible sdformat options here if necessary
pkg_search_module(SDFORMAT REQUIRED sdformat sdformat9 sdformat12 sdformat14 sdformat15)

if(ROCK_QT_VERSION_4)
    rock_vizkit_plugin(robot_model-viz
        RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp OSGSegment.cpp VBOVisitor.hpp
        DEPS_PKGCONFIG kdl_parser base-types base-viz base-lib base-logging ${SDFORMAT_MODULE_NAME} urdfdom
        MOC RobotVisualization.hpp
        HEADERS RobotVisualization.hpp RobotModel.h OSGSegment.h VBOVisitor.hpp)
endif(ROCK_QT_VERSION_4)

if(ROCK_QT_VERSION_5)
    rock_vizkit_plugin_qt5(robot_model-viz-qt5
        RobotVisualization.cpp RobotModel.cpp OSGHelpers.hpp OSGSegment.cpp VBOVisitor.hpp
        DEPS_PKGCONFIG kdl_parser base-types base-viz-qt5 base-lib base-logging ${SDFORMAT_MODULE_NAME} urdfdom
        MOC5 RobotVisualization.hpp
        HEADERS RobotVisualization.hpp RobotModel.h OSGSegment.h VBOVisitor.hpp)
endif(ROCK_QT_VERSION_5)
