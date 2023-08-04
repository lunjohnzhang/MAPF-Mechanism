project(drone)

cmake_minimum_required (VERSION 2.8.12)

IF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE "DEBUG")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /usr/local/include/parlay")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(drone ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(Parlay REQUIRED)

include_directories( ${Boost_INCLUDE_DIRS} "/usr/local/include/parlay")
target_link_libraries(drone ${Boost_LIBRARIES} Parlay::parlay)
