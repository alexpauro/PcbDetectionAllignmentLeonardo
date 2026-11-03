add_executable(pcb_app
    src/main.cpp
)

target_link_libraries(pcb_app
    PRIVATE
        pcb::calibration
        ${OpenCV_LIBS}
)

set_target_properties(pcb_app PROPERTIES
    OUTPUT_NAME "pcb_detection_alignment"
)

