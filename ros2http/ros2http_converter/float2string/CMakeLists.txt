project(float2string)

cmake_minimum_required(VERSION 2.4.6)

include_directories(${PROJECT_SOURCE_DIR}/src)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(float2string SHARED src/float_string.c)
add_executable(test src/main.c)
