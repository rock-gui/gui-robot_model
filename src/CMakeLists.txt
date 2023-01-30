rock_find_qt5(REQUIRED Gui Core Widgets)
rock_executable(rock_roboviz Main.cpp
    DEPS_PKGCONFIG
        vizkit3d
        eigen3
        sdformat
        robot_model-viz
    LIBS Qt5::Core Qt5::Widgets Qt5::Gui
    NOINSTALL)
