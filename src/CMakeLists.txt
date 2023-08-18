rock_find_qt5(REQUIRED Gui Core Widgets)
rock_executable(rock_roboviz Main.cpp
    DEPS_PKGCONFIG
        vizkit3d
        eigen3
        sdformat
        control_ui
        base-types
    LIBS Qt5::Core Qt5::Widgets Qt5::Gui robot_model-viz
    NOINSTALL)
