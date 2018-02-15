cmake_minimum_required (VERSION 2.8)
project (sonarlog_target_tracking)

option (COMPILE_EXTRACTION          OFF)
option (COMPILE_VIDEOCONV           OFF)
option (COMPILE_PREPROCESSING       OFF)
option (COMPILE_DETECTION           OFF)
option (COMPILE_EVALUATION          OFF)

set ( COMPILE_EXTRACTION      ON )
set ( COMPILE_VIDEOCONV       ON )
set ( COMPILE_PREPROCESSING   ON )
set ( COMPILE_TRAINING        ON )
set ( COMPILE_DETECTION       ON )
set ( COMPILE_EVALUATION      ON )

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

if (COMPILE_EXTRACTION)
    message ( STATUS "Compile sonarlog-extraction example")

    add_executable (
        sonarlog-extraction
        examples/extraction.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-extraction
        ${LIBS}
    )

    install (
        TARGETS sonarlog-extraction
        DESTINATION bin
    )
endif(COMPILE_CONVERT_TO_IMAGES)

if (COMPILE_VIDEOCONV)
    message ( STATUS "Compile videoconv example")

    add_executable (
        sonarlog-videoconv
        examples/videoconv.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-videoconv
        ${LIBS}
    )

    install (
        TARGETS sonarlog-videoconv
        DESTINATION bin
    )
endif(COMPILE_VIDEOCONV)

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

if (COMPILE_EVALUATION)

    message ( STATUS "Compile Evaluation example")

    add_executable (
        sonarlog-evaluation
        examples/evaluation.cpp
        ${SOURCES}
    )

    target_link_libraries (
        sonarlog-evaluation
        annotation_filereader
        ${LIBS}
    )

    install (
        TARGETS sonarlog-evaluation
        DESTINATION bin
    )

endif(COMPILE_EVALUATION)
