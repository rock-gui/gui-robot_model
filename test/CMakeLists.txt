rock_find_qt5(REQUIRED Gui Core OpenGL)

find_package(Boost COMPONENTS system filesystem REQUIRED)
rock_executable(plugintest
                    SOURCES plugintest.cpp ${timer_MOC} my_timer.h
                    MOC my_timer.h
                    DEPS robot_model-viz
                         Qt5::Widgets Qt5::OpenGL Qt5::Gui
)
target_link_libraries(plugintest
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)


