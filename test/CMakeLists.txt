FIND_PACKAGE(Qt4 REQUIRED QtCore QtGui QtOpenGL)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#find_package(PkgConfig)
#pkg_check_modules(EIGEN eigen3)
#include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Boost COMPONENTS system filesystem REQUIRED)
rock_executable(plugintest
                    SOURCES plugintest.cpp ${timer_MOC} my_timer.h
                    MOC my_timer.h
                    DEPS_PKGCONFIG QtCore QtGui QtOpenGL
                    DEPS robot_model-viz)
target_link_libraries(plugintest
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)


