add_library(output_processing
    src/output_processor.cpp
)

target_include_directories(output_processing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(output_processing
    PUBLIC
        pcb::component_detection
        pcb::cv_component_detection
        ${OpenCV_LIBS}
)

add_library(pcb::output_processing ALIAS output_processing)

