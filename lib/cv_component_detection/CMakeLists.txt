add_library(cv_component_detection
    src/cv_component_detector.cpp
)

target_include_directories(cv_component_detection
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(cv_component_detection
    PUBLIC
        ${OpenCV_LIBS}
)

add_library(pcb::cv_component_detection ALIAS cv_component_detection)

