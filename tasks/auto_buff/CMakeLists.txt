cmake_minimum_required(VERSION 3.16)
# OpenVINO 路径由根 CMakeLists.txt 统一解析（2025.4.0 优先，回退 2024.6.0）

find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime ${CERES_LIBRARIES})
