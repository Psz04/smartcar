# 包含交叉编译工具链设置
include(cross.cmake)

cmake_minimum_required(VERSION 3.5)
project(project VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)

find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    videoio
    highgui
    imgcodecs
)
message(STATUS "OpenCV found in: ${OpenCV_DIR}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/driver/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/app/inc)
include_directories(/home/smartcar/boost)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ZF/zf_common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ZF/zf_components)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ZF/zf_device)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ZF/zf_driver)

# ncnn 头文件与库（龙邱 lqncnn）
set(NCNN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ZF/zf_components/lqncnn/ncnn_install)
include_directories(${NCNN_DIR}/include)
include_directories(${NCNN_DIR}/include/ncnn)
link_directories(${NCNN_DIR}/lib)

aux_source_directory(driver/src SRC)
aux_source_directory(app/src SRC)
aux_source_directory(ZF/zf_common SRC)
aux_source_directory(ZF/zf_components SRC)
aux_source_directory(ZF/zf_device SRC)
aux_source_directory(ZF/zf_driver SRC)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ncnn
    pthread
    m
)