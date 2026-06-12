cmake_minimum_required(VERSION 3.16)
project(test_decoder C)

set(CMAKE_C_STANDARD 99)

# Include decoder headers from main
include_directories(../main)

# Add compilation target linking tests and decoder sources
add_executable(test_decoder test_decoder.c ../main/decoder.c)
