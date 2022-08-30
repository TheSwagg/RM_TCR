cmake_minimum_required(VERSION 3.5)
project(RM)

# ����C++ 11 ��׼֧��
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Eigen��·��(eigen3���ڵ�λ��)
set(EIGEN_DIR "/usr/include/eigen3")
# ����Eigenͷ�ļ�
include_directories(${EIGEN_DIR})

# Ѱ��Opencv��
find_package(OpenCV REQUIRED)
# ����ͷ�ļ�
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
# ����Opencv��
target_link_libraries(RM ${OpenCV_LIBS} gxiapi)

# ���include files
include_directories(./header)