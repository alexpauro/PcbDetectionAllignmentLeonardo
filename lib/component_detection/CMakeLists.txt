add_library(component_detection
    src/component_detector.cpp
    src/class_mapper.cpp
    src/multi_model_detector.cpp
)

target_include_directories(component_detection
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(component_detection
    PUBLIC
        pcb::common
        ${OpenCV_LIBS}
)

add_library(pcb::component_detection ALIAS component_detection)

