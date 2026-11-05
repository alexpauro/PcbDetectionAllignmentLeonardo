add_library(component_alignment
    src/component_alignment.cpp
)

target_include_directories(component_alignment
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(component_alignment
    PUBLIC
        pcb::common
        pcb::aoi_verification
        pcb::output_processing
        pcb::board_alignment
        pcb::cv_component_detection
        ${OpenCV_LIBS}
)

add_library(pcb::component_alignment ALIAS component_alignment)
