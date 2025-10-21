cmake_minimum_required(VERSION 3.15)
project(FuelOptimizedRouting)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Homebrew paths (adjust for Intel vs Apple Silicon)
if(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(HOMEBREW_PREFIX /opt/homebrew)
    else()
        set(HOMEBREW_PREFIX /usr/local)
    endif()
endif()

# MacPorts path for protozero
set(MACPORTS_PREFIX /opt/local)

# Include and link directories
include_directories(
        ${HOMEBREW_PREFIX}/include
        ${MACPORTS_PREFIX}/include           # <-- Add this line
)

link_directories(
        ${HOMEBREW_PREFIX}/lib
        ${MACPORTS_PREFIX}/lib               # <-- Optional, in case you link other MacPorts libs
)

# Find Boost (required)
find_package(Boost REQUIRED COMPONENTS system filesystem)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

# Add your executable
add_executable(FuelOptimizedRouting Driving.cpp Vehicle.cpp)

# Link against required libraries
target_link_libraries(FuelOptimizedRouting
        ${Boost_LIBRARIES}
        z bz2 expat
)




