cmake_minimum_required (VERSION 2.8)
project (sonarlog_target_tracking)

include ( "${WORKSPACE_DIR}/base/cmake/Dependencies.cmake"  )
include ( "${WORKSPACE_DIR}/third_party/cmake/Gnuplot.cmake"  )

option (COMPILE_FIRST_SAMPLE_PROC       OFF)
option (COMPILE_CONVERT_TO_IMAGES       OFF)
option (COMPILE_DFT_PROC                OFF)
option (COMPILE_VIDEO_GENERATOR         OFF)
option (COMPILE_NEAREST_NEIGHBOR        OFF)
option (COMPILE_POLAR_PROCESSING        OFF)
option (COMPILE_CARTESIAN_PROCESSING    OFF)

add_project_dependency (base                  ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_project_dependency (rock_util             ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_project_dependency (sonar_processing      ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_project_dependency (sonarlog_annotation   ${WORKSPACE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

setup_headers ( sonar_processing ${sonar_processing_SOURCE_DIR} )
setup_headers ( ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/src" )

include ( FindPkgConfig  )
include ( BoostTest )

find_package ( Boost COMPONENTS system filesystem program_options iostreams unit_test_framework REQUIRED )
find_package ( Eigen3 REQUIRED )
find_package ( OpenCV REQUIRED )

set (
    ${PROJECT_NAME}_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/include
    CACHE PATH "Sonar's log Target Tracking Include Directory"
    FORCE
)

file ( GLOB SRCS RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/[^main]*.cpp )
# list ( REMOVE_ITEM SRCS "src/main.cpp" )

set ( SRCS ${SRCS} ${sonar_processing_SOURCES} )

pkg_check_modules(pocolog_cpp pocolog_cpp)

include_directories (
    ${sonarlog_target_tracking_INCLUDE_DIR}
    ${base_INCLUDE_DIR}
    ${sonar_util_INCLUDE_DIR}
    ${rock_util_INCLUDE_DIR}
    ${sonar_processing_INCLUDE_DIR}
    ${gnuplot_INCLUDE_DIR}
    ${ROCK_ROBOTICS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories (
    ${ROCK_ROBOTICS_LIBRARY_DIR}
)

set (
    LIBS
    rock_util
    base
    ${Boost_LIBRARIES}
    ${pocolog_cpp_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable (
    sonarlog_target_tracking
    src/main.cpp
    ${SRCS}
)

target_link_libraries (
    sonarlog_target_tracking
    ${LIBS}
)

if (COMPILE_FIRST_SAMPLE_PROC)
    message ( STATUS "Compile first_sample_proc example")

    add_executable (
        first_sample_proc
        examples/first_sample_proc.cpp
        ${SRCS}
    )

    target_link_libraries (
        first_sample_proc
        ${LIBS}
    )
endif(COMPILE_FIRST_SAMPLE_PROC)

if (COMPILE_CONVERT_TO_IMAGES)
    message ( STATUS "Compile first_sample_proc example")

    add_executable (
        convert_to_images
        examples/convert_to_images.cpp
        ${SRCS}
    )

    target_link_libraries (
        convert_to_images
        ${LIBS}
    )

endif(COMPILE_CONVERT_TO_IMAGES)

if (COMPILE_DFT_PROC)
    message ( STATUS "Compile dft_proc example")

    add_executable (
        dft_proc
        examples/dft_proc.cpp
        ${SRCS}
    )

    target_link_libraries (
        dft_proc
        ${LIBS}
    )

endif(COMPILE_DFT_PROC)

if (COMPILE_VIDEO_GENERATOR)
    message ( STATUS "Compile video_generator example")

    add_executable (
        video_generator
        examples/video_generator.cpp
        ${SRCS}
    )

    target_link_libraries (
        video_generator
        ${LIBS}
    )

endif(COMPILE_VIDEO_GENERATOR)

if (COMPILE_NEAREST_NEIGHBOR)
    message ( STATUS "Compile nearest_neighbor example")

    add_executable (
        nearest_neighbor
        examples/nearest_neighbor.cpp
        ${SRCS}
    )

    target_link_libraries (
        nearest_neighbor
        ${LIBS}
    )

endif(COMPILE_NEAREST_NEIGHBOR)

if (COMPILE_POLAR_PROCESSING)

    message ( STATUS "Compile Polar Cart Processing example")

    add_executable (
        polar_processing
        examples/polar_processing.cpp
        ${SRCS}
    )

    target_link_libraries (
        polar_processing
        ${LIBS}
    )

endif(COMPILE_POLAR_PROCESSING)

if (COMPILE_CARTESIAN_PROCESSING)

    message ( STATUS "Compile Cartesian Processing example")

    add_executable (
        cartesian_processing
        examples/cartesian_processing_main.cpp
        examples/cartesian_processing.cpp
        ${SRCS}
    )

    target_link_libraries (
        cartesian_processing
        annotation_filereader
        ${LIBS}
    )

endif(COMPILE_CARTESIAN_PROCESSING)

boost_test_init()

add_boost_test (
    test_ArgumentParser
    SOURCES test/test_ArgumentParser.cpp src/ArgumentParser.cpp
    LIBRARIES ${Boost_LIBRARIES}
)

configure_file (
    ${PROJECT_SOURCE_DIR}/scripts/example0.sh.in
    ${PROJECT_BINARY_DIR}/scripts/example0.sh
    @ONLY
)

configure_file (
    ${PROJECT_SOURCE_DIR}/scripts/example1.sh.in
    ${PROJECT_BINARY_DIR}/scripts/example1.sh
    @ONLY
)
