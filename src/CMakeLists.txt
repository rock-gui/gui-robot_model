rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL Widgets)

if(ROCK_QT_VERSION_4)
    rock_executable(rock_roboviz Main.cpp
        DEPS_PKGCONFIG
            vizkit3d
            control_ui
            base-types
        LIBS robot_model-viz
        NOINSTALL)
    if(ROCK_QT_VERSION_5)
        rock_executable(rock_roboviz-qt5 Main.cpp
            DEPS_PKGCONFIG
                vizkit3d-qt5
                control_ui-qt5
                base-types
            LIBS robot_model-viz-qt5
            DEPS Qt5::Widgets
            NOINSTALL)
    endif()
elseif(ROCK_QT_VERSION_5)
    rock_executable(rock_roboviz Main.cpp
        DEPS_PKGCONFIG
            vizkit3d-qt5
            control_ui-qt5
            base-types
        LIBS robot_model-viz-qt5
        DEPS Qt5::Widgets
        NOINSTALL)
else()
    message(FATAL_ERROR "Could not find either qt4 or qt5")
endif()
