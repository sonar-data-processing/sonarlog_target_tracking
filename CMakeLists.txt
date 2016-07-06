cmake_minimum_required (VERSION 2.6)
project (sonarlog_target_tracking)

include ( "${WORKSPACE_DIR}/base/cmake/Dependencies.cmake"  )

add_project_dependency (base                  ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_project_dependency (rock_util             ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_project_dependency (sonar_target_tracking ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include ( FindPkgConfig  )
include ( BoostTest )

find_package ( Boost COMPONENTS system filesystem program_options unit_test_framework REQUIRED )
find_package ( Eigen3 REQUIRED )
find_package ( OpenCV REQUIRED )

set ( ${PROJECT_NAME}_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include CACHE INTERNAL "Sonar's log Target Tracking Include Directory" FORCE )

file ( GLOB SRCS RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/*.cpp )

pkg_check_modules(pocolog_cpp pocolog_cpp)

include_directories (
    ${sonarlog_target_tracking_INCLUDE_DIR}
    ${base_INCLUDE_DIR}
    ${sonar_util_INCLUDE_DIR}
    ${rock_util_INCLUDE_DIR}
    ${sonar_target_tracking_INCLUDE_DIR}
    ${ROCK_ROBOTICS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories (
    ${ROCK_ROBOTICS_LIBRARY_DIR}
)

add_executable (
    sonarlog_target_tracking
    ${SRCS}
)

target_link_libraries (
    sonarlog_target_tracking
    rock_util
    sonar_target_tracking
    ${Boost_LIBRARIES}
    ${pocolog_cpp_LIBRARIES}
    ${OpenCV_LIBS}
)

boost_test_init()

add_boost_test (
    test_ArgumentParser 
    SOURCES test/test_ArgumentParser.cpp src/ArgumentParser.cpp
    LIBRARIES ${Boost_LIBRARIES}
)

configure_file (
    ${PROJECT_SOURCE_DIR}/scripts/example_ferry.sh.in
    ${PROJECT_BINARY_DIR}/scripts/example_ferry.sh
    @ONLY
)
