cmake_minimum_required(VERSION 3.5)
project(RM)

# 添加C++ 11 标准支持
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Eigen库路径(eigen3所在的位置)
set(EIGEN_DIR "/usr/include/eigen3")
# 添加Eigen头文件
include_directories(${EIGEN_DIR})

# 寻找Opencv库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp
        header/General.h
        header/Armor.h
        header/Filter.h
        header/AngleSolver_2.h
        header/ImageProcess.h
        header/RemoteController.h
        header/serialport.h
        Armor_recognition/LightBar.cpp
        Armor_recognition/findLights.cpp
        Armor_recognition/ArmorBox.cpp
        Armor_recognition/ArmorDetector.cpp
        Armor_recognition/ArmorNumClassifier.cpp
        Armor_recognition/matchArmors.cpp
        AngleSolver/Filter.cpp
        AngleSolver/AngleSolver_2.cpp
        global/serialport.cpp
        global/General.cpp
        Thread/ImageProcess.cpp
        Thread/RemoteController.cpp
        DaHengCamera/DaHengCamera.h
        DaHengCamera/DaHengCamera.cpp
        buff_include/FindBuff.h
        buff_include/BuffAngleSolver.h
        buff_src/FindBuff.cpp
        buff_src/BuffAngleSolver.cpp
        )
add_executable(RM ${SOURCE_FILES} header/General.h header/Armor.h header/Filter.h header/AngleSolver_2.h header/ImageProcess.h header/RemoteController.h header/serialport.h Armor_recognition/LightBar.cpp Armor_recognition/findLights.cpp Armor_recognition/ArmorBox.cpp Armor_recognition/ArmorDetector.cpp Armor_recognition/ArmorNumClassifier.cpp Armor_recognition/matchArmors.cpp AngleSolver/Filter.cpp AngleSolver/AngleSolver_2.cpp global/serialport.cpp global/General.cpp Thread/ImageProcess.cpp Thread/RemoteController.cpp DaHengCamera/DaHengCamera.h DaHengCamera/DaHengCamera.cpp)
# 链接Opencv库
target_link_libraries(RM ${OpenCV_LIBS} gxiapi)

# 大恒include files
include_directories(./header)