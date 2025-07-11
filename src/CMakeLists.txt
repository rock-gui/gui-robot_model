rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL Widgets)

add_definitions(-DOROCOS_TARGET=gnulinux)

find_package(Boost COMPONENTS program_options REQUIRED )


if(ROCK_QT_VERSION_4)
    rock_executable(rock-roboviz_bin Main.cpp
        DEPS_PKGCONFIG
            vizkit3d
            control_ui
            base-types
            orocos_cpp
        LIBS robot_model-viz
        DEPS Boost::program_options
        )
    if(ROCK_QT_VERSION_5)
        rock_executable(rock-roboviz_bin-qt5 Main.cpp
            DEPS_PKGCONFIG
                vizkit3d-qt5
                control_ui-qt5
                base-types
                orocos_cpp
            LIBS robot_model-viz-qt5
            DEPS Qt5::Widgets Boost::program_options
            )
    endif()
elseif(ROCK_QT_VERSION_5)
    rock_executable(rock-roboviz_bin Main.cpp
        DEPS_PKGCONFIG
            vizkit3d-qt5
            control_ui-qt5
            base-types
            orocos_cpp
        LIBS robot_model-viz-qt5
        DEPS Qt5::Widgets Boost::program_options
        )
else()
    message(FATAL_ERROR "Could not find either qt4 or qt5")
endif()
