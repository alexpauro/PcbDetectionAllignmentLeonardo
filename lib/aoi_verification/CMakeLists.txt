add_library(aoi_verification
    src/aoi_verification.cpp
)

target_include_directories(aoi_verification
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(aoi_verification
    PUBLIC
        pcb::output_processing
        pcb::board_alignment
        pcb::component_detection
        pcb::cv_component_detection
        ${OpenCV_LIBS}
)

add_library(pcb::aoi_verification ALIAS aoi_verification)

