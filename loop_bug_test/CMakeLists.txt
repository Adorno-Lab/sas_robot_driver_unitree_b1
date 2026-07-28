cmake_minimum_required(VERSION 3.16)

project(loop_bug_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
 set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
 set(ARCH arm64)
endif()
set(UNITREE_LEGGED_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/unitree_legged_sdk/)
include_directories(${UNITREE_LEGGED_SDK_DIR}/include/)
link_directories(${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH})
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

add_library(dummy_driver dummy_driver.h dummy_driver.cpp)
target_link_libraries(dummy_driver
${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.so)

add_executable(loop_bug_test main.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.so
    dummy_driver
)
