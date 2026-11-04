add_executable(pcb_app
    src/main.cpp
)

target_link_libraries(pcb_app
    PRIVATE
        pcb::calibration
        pcb::board_alignment
        pcb::component_detection
        pcb::cv_component_detection
        pcb::output_processing
        pcb::aoi_verification
        ${OpenCV_LIBS}
)

set_target_properties(pcb_app PROPERTIES
    OUTPUT_NAME "pcb_detection_alignment"
)

add_executable(test_aoi_verification
    src/test_aoi_verification.cpp
)

target_link_libraries(test_aoi_verification
    PRIVATE
        pcb::aoi_verification
        pcb::output_processing
        pcb::board_alignment
        ${OpenCV_LIBS}
)

