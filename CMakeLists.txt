cmake_minimum_required (VERSION 2.8)
project (sonarlog_target_tracking)

option (COMPILE_CONVERT_TO_IMAGES   OFF)
option (COMPILE_DFT_PROC            OFF)
option (COMPILE_VIDEO_GENERATOR     OFF)
option (COMPILE_PREPROCESSING       OFF)
option (COMPILE_DETECTION           OFF)
option (COMPILE_EVALUATION          OFF)

set ( COMPILE_CONVERT_TO_IMAGES OFF )
set ( COMPILE_DFT_PROC          OFF )
set ( COMPILE_VIDEO_GENERATOR   OFF )
set ( COMPILE_PREPROCESSING      ON )
set ( COMPILE_TRAINING           ON )
set ( COMPILE_DETECTION          ON )

list(APPEND CMAKE_MODULE_PATH $ENV{SONAR_TOOLKIT_PREFIX}/base/cmake)

include ( FindPkgConfig  )

find_package ( Boost COMPONENTS system filesystem program_options iostreams unit_test_framework REQUIRED )
find_package ( Eigen3 REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( yaml-cpp )

file ( GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp )
file ( GLOB HEADERS ${PROJECT_SOURCE_DIR}/src/*.hpp )

pkg_check_modules(pocolog_cpp pocolog_cpp)

include_directories (
    ${PROJECT_SOURCE_DIR}/src
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/sonar_toolkit
    ${EIGEN3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

link_directories (
    ${CMAKE_INSTALL_PREFIX}/lib
    ${CMAKE_INSTALL_PREFIX}/lib/sonar_toolkit
)

set (
    LIBS
    sonar_processing
    annotation_filereader
    sonar_util
    rock_util
    base
    ${Boost_LIBRARIES}
    ${pocolog_cpp_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
)

add_library (
    sonarlog_target_tracking SHARED
    ${SOURCES}
)

target_link_libraries (
    sonarlog_target_tracking
    ${LIBS}
)

install (
    FILES ${HEADERS}
    DESTINATION include/sonar_toolkit/${PROJECT_NAME}
)

if (COMPILE_CONVERT_TO_IMAGES)
    message ( STATUS "Compile sonarlog-convert_to_images example")

    add_executable (
        sonarlog-convert_to_images
        examples/convert_to_images.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-convert_to_images
        ${LIBS}
    )

    install (
        TARGETS sonarlog-convert_to_images
        DESTINATION bin
    )
endif(COMPILE_CONVERT_TO_IMAGES)

if (COMPILE_DFT_PROC)
    message ( STATUS "Compile DFT processing example")

    add_executable (
        sonarlog-dft_proc
        examples/dft_proc.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-dft_proc
        ${LIBS}
    )

    install (
        TARGETS sonarlog-dft_proc
        DESTINATION bin
    )
endif(COMPILE_DFT_PROC)

if (COMPILE_VIDEO_GENERATOR)
    message ( STATUS "Compile video_generator example")

    add_executable (
        sonarlog-video_generator
        examples/video_generator.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-video_generator
        ${LIBS}
    )

    install (
        TARGETS sonarlog-video_generator
        DESTINATION bin
    )
endif(COMPILE_VIDEO_GENERATOR)

if (COMPILE_DETECTION)

    message ( STATUS "Compile Detection example")

    add_executable (
        sonarlog-detection
        examples/detection.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-detection
        annotation_filereader
        sonarlog_target_tracking
        ${LIBS}
    )

    install (
        TARGETS sonarlog-detection
        DESTINATION bin
    )
endif(COMPILE_DETECTION)

if (COMPILE_PREPROCESSING)

    message ( STATUS "Compile Preprocessing example")

    add_executable (
        sonarlog-preprocessing
        examples/preprocessing.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-preprocessing
        annotation_filereader
        ${LIBS}
    )

    install (
        TARGETS sonarlog-preprocessing
        DESTINATION bin
    )

endif(COMPILE_PREPROCESSING)

if (COMPILE_TRAINING)

    message ( STATUS "Compile Training example")

    add_executable (
        sonarlog-training
        examples/training.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-training
        annotation_filereader
        ${LIBS}
    )

    install (
        TARGETS sonarlog-training
        DESTINATION bin
    )

endif(COMPILE_TRAINING)
