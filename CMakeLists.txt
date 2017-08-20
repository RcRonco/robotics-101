cmake_minimum_required(VERSION 3.6)
project(Robotics-101)

set(CMAKE_CXX_STANDARD 14)

set(SRC_FILES main.cpp Constants.h MovementManager.h MovementManager.cpp PathPlanner.h PathPlanner.cpp Robot.h Robot.cpp Structs.h)
set(GUI_SRC Gui/MapDrawer.h Gui/MapDrawer.cpp Gui/MapPointType.cpp)
set(CONFIG_SRC Configuration/ParameterSerialization.h Configuration/ParameterSerialization.cpp)
set(LCL_SRC Localization/LocalizationManager.h Localization/LocalizationManager.cpp Localization/Particle.h)
set(NODE_SRC NodeMap/Node.h NodeMap/NodeMap.h NodeMap/NodeMap.cpp)
set(UTILS_SRC Utils/AngleUtils.h Utils/AngleUtils.cpp Utils/PositionUtils.h Utils/PositionUtils.cpp)

add_executable(Robotics-101 ${SRC_FILES} ${GUI_SRC} ${CONFIG_SRC} ${LCL_SRC} ${NODE_SRC} ${UTILS_SRC})