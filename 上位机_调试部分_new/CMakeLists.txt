cmake_minimum_required(VERSION 3.10)

project(project VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 默认使用 Debug，便于在 VSCode 中打断点调试。
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "构建类型" FORCE)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
message(STATUS "OpenCV found in: ${OpenCV_DIR}")

# ---------------------------------------------------------------------------
# 自动定位从车端复制过来的 image.cpp / image.h。
#
# 支持把两个文件一起放到本目录，或分别放到 src/、lib/、include/、app/inc/。
# 也支持通过 -DIMAGE_CPP=... -DIMAGE_H=... 显式指定。
# ---------------------------------------------------------------------------
if(NOT IMAGE_CPP)
    set(_image_cpp_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/image.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/image.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/app/src/image.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/image.cpp"
    )

    foreach(_candidate IN LISTS _image_cpp_candidates)
        if(EXISTS "${_candidate}")
            set(IMAGE_CPP "${_candidate}")
            break()
        endif()
    endforeach()
endif()

if(NOT IMAGE_CPP OR NOT EXISTS "${IMAGE_CPP}")
    message(FATAL_ERROR
        "未找到 image.cpp。\n"
        "请从 校内赛图像控制代码-new/app/src/image.cpp 复制到本目录，\n"
        "或复制到 src/、app/src/，或使用 -DIMAGE_CPP=/绝对/路径/image.cpp 指定。")
endif()

if(NOT IMAGE_H)
    set(_image_h_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/image.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/image.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/image.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/image.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/app/inc/image.h"
    )

    foreach(_candidate IN LISTS _image_h_candidates)
        if(EXISTS "${_candidate}")
            set(IMAGE_H "${_candidate}")
            break()
        endif()
    endforeach()
endif()

if(NOT IMAGE_H OR NOT EXISTS "${IMAGE_H}")
    message(FATAL_ERROR
        "未找到 image.h。\n"
        "请从 校内赛图像控制代码-new/app/inc/image.h 复制到本目录，\n"
        "或复制到 lib/、include/、app/inc/，或使用 -DIMAGE_H=/绝对/路径/image.h 指定。")
endif()

get_filename_component(IMAGE_H_DIR "${IMAGE_H}" DIRECTORY)

message(STATUS "Debug image.cpp: ${IMAGE_CPP}")
message(STATUS "Debug image.h:   ${IMAGE_H}")

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/cameratest.cpp
    src/camera_driver.cpp
    ${IMAGE_CPP}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${IMAGE_H_DIR}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
