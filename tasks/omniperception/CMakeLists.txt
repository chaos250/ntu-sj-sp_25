cmake_minimum_required(VERSION 3.16)

# OpenVINO 路径由根 CMakeLists.txt 统一解析（2025.4.0 优先，回退 2024.6.0）

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)
