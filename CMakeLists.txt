cmake_minimum_required (VERSION 2.8)
project (sonarlog_target_tracking)

include ( "${WORKSPACE_DIR}/base/cmake/Dependencies.cmake"  )
include ( "${WORKSPACE_DIR}/third_party/cmake/Gnuplot.cmake"  )

option (COMPILE_CONVERT_TO_IMAGES   OFF)
option (COMPILE_DFT_PROC            OFF)
option (COMPILE_VIDEO_GENERATOR     OFF)
option (COMPILE_PREPROCESSING       OFF)
option (COMPILE_DETECTION           OFF)
option (COMPILE_EVALUATION          OFF)

set ( COMPILE_PREPROCESSING ON )
set ( COMPILE_DETECTION     ON )
set ( COMPILE_EVALUATION    ON )

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
find_package ( yaml-cpp )

set (
    ${PROJECT_NAME}_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/include
    CACHE PATH "Sonar's log Target Tracking Include Directory"
    FORCE
)

file ( GLOB SRCS RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/[^main]*.cpp )

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
    ${YAML_CPP_INCLUDE_DIR}
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
    ${YAML_CPP_LIBRARIES}
)

add_library (
    sonarlog_target_tracking STATIC
    ${SRCS}
)

target_link_libraries (
    sonarlog_target_tracking
    ${LIBS}
)

add_executable (
    sonarlog-target-tracking
    src/main.cpp
)

target_link_libraries (
    sonarlog-target-tracking
    ${LIBS}
    sonarlog_target_tracking
)

if (COMPILE_CONVERT_TO_IMAGES)
    message ( STATUS "Compile first_sample_proc example")

    add_executable (
        sonarlog-convert_to_images
        examples/convert_to_images.cpp
        ${SRCS}
    )

    target_link_libraries (
        sonarlog-convert_to_images
        ${LIBS}
    )

endif(COMPILE_CONVERT_TO_IMAGES)

if (COMPILE_DFT_PROC)
    message ( STATUS "Compile DFT processing example")

    add_executable (
        sonarlog-dft_proc
        examples/dft_proc.cpp
        ${SRCS}
    )

    target_link_libraries (
        sonarlog-dft_proc
        ${LIBS}
    )

endif(COMPILE_DFT_PROC)

if (COMPILE_VIDEO_GENERATOR)
    message ( STATUS "Compile video_generator example")

    add_executable (
        sonarlog-video_generator
        examples/video_generator.cpp
        ${SRCS}
    )

    target_link_libraries (
        sonarlog-video-generator
        ${LIBS}
    )

endif(COMPILE_VIDEO_GENERATOR)

if (COMPILE_DETECTION)

    message ( STATUS "Compile Detection example")

    add_executable (
        sonarlog-detection
        examples/detection.cpp
        ${SRCS}
    )

    target_link_libraries (
        sonarlog-detection
        annotation_filereader
        sonarlog_target_tracking
        ${LIBS}
    )

endif(COMPILE_DETECTION)

if (COMPILE_PREPROCESSING)

    message ( STATUS "Compile Preprocessing example")

    add_executable (
        sonarlog-preprocessing
        examples/preprocessing.cpp
        ${SRCS}
    )

    target_link_libraries (
        sonarlog-preprocessing
        annotation_filereader
        ${LIBS}
    )

endif(COMPILE_PREPROCESSING)

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
