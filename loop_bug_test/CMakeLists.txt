cmake_minimum_required(VERSION 3.16)
project(loop_bug_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------- Sanitizer options (mutually exclusive) --------------------
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

# Ensure only one sanitizer is enabled at a time
if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "Cannot enable both AddressSanitizer and ThreadSanitizer at the same time!")
endif()

# -------------------- Architecture detection --------------------
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    set(ARCH arm64)
endif()

# -------------------- Unitree SDK paths --------------------
set(UNITREE_LEGGED_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/unitree_legged_sdk/)
include_directories(${UNITREE_LEGGED_SDK_DIR}/include/)
link_directories(${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH})

# -------------------- Compiler / linker flags --------------------
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# Add sanitizer flags if enabled
if(ENABLE_ASAN)
    # AddressSanitizer flags
    set(SAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -g -O1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SAN_FLAGS}")
    message(STATUS "AddressSanitizer enabled")

    # Optional: Detect memory leaks
    set(ASAN_OPTIONS "detect_leaks=1:verbosity=1" CACHE STRING "ASan options")
    message(STATUS "ASAN_OPTIONS=${ASAN_OPTIONS}")
endif()

if(ENABLE_TSAN)
    # ThreadSanitizer flags
    set(SAN_FLAGS "-fsanitize=thread -fno-omit-frame-pointer -g -O1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SAN_FLAGS}")
    message(STATUS "ThreadSanitizer enabled")

    # Optional: TSan options
    set(TSAN_OPTIONS "history_size=7:detect_deadlocks=1" CACHE STRING "TSan options")
    message(STATUS "TSAN_OPTIONS=${TSAN_OPTIONS}")
endif()

# -------------------- Targets --------------------
add_library(dummy_driver dummy_driver.h dummy_driver.cpp)
target_link_libraries(dummy_driver
    ${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.so
)

add_executable(loop_bug_test main.cpp)
target_link_libraries(loop_bug_test
    ${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.so
    dummy_driver
)
