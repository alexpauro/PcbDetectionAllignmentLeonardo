add_library(component_segmentation
    src/component_segmentation.cpp
)

target_include_directories(component_segmentation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(component_segmentation
    PUBLIC
        pcb::common
        pcb::aoi_verification
        pcb::output_processing
        pcb::board_alignment
        pcb::cv_component_detection
        ${OpenCV_LIBS}
)

add_library(pcb::component_segmentation ALIAS component_segmentation)

