cmake_minimum_required(VERSION 3.18)
project(KHI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3 -v")

# Enable sanitizers if supported
option(ENABLE_SANITIZERS "Enable Address and Undefined Behavior Sanitizers" OFF)

if(ENABLE_SANITIZERS)
    set(SANITIZER_FLAGS "-fsanitize=address,undefined -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZER_FLAGS}")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${SANITIZER_FLAGS}")
endif()

add_executable(KHI main.cpp
        timer.h
        data.h
        data.cpp
        kd_tree.h
        kd_tree.cpp
        ma_graph.h
        ma_graph.cpp
        query.h
        query.cpp)

target_include_directories(KHI PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
