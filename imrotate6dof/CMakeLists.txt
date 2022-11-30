cmake_minimum_required(VERSION 3.0.0)
project(imRotate6DoF VERSION 1.1.0)

#find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
add_executable(imRotate6DoF imRotate6DoF.cpp)
#target_link_libraries(imRotate6DoF ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
