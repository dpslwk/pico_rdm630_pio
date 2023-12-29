cmake_minimum_required(VERSION 3.12)

OPTION(EXAMPLE_BUILD "Build all examples" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(EXAMPLE_BUILD)
  set(NAME example_test_rdm630)

  if (NOT EXISTS $ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Pico SDK Path not found, please set correct environment variable PICO_SDK_PATH")
  endif ()

  include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

  project(${NAME} C CXX ASM)

  set(CMAKE_C_STANDARD 11)
  set(CMAKE_CXX_STANDARD 17)

  pico_sdk_init()

  add_subdirectory(src) # the library
  add_subdirectory(example) # the example
else()
  add_subdirectory(src)
endif()
