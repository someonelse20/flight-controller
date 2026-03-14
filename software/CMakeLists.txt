cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once, and is not re-generated if converter is
# called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME flight-controller)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(
	${CMAKE_PROJECT_NAME} PRIVATE # Add user defined library search paths
	Core/Inc/sensor.h
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE # Add user sources here
	Core/Src/sensor.c

	Lib/ahrs/ahrs.c
	Lib/ahrs/ahrs_math.c
	Lib/pid/pid.c
	Lib/lsm6dso-pid/lsm6dso_reg.c
	Lib/MMC5983-STM32-HAL-Driver/MMC5983.c
	Lib/bmp384/src/driver_bmp384.c
	Lib/STM32-GNSS/GNSS.c
)

# Add include paths
target_include_directories(
	${CMAKE_PROJECT_NAME} PRIVATE # Add user defined include paths
	Lib/ahrs
	Lib/pid
	Lib/lsm6dso-pid
	Lib/MMC5983-STM32-HAL-Driver
	Lib/bmp384/src
	Lib/bmp384/interface
	Lib/STM32-GNSS
)

# Add project symbols (macros)
target_compile_definitions(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(
  ${CMAKE_PROJECT_NAME} stm32cubemx # Add user defined libraries
)
