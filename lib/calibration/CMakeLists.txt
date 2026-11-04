add_library(calibration
    src/calibration.cpp
    src/undistort.cpp
)

target_include_directories(calibration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(calibration
    PUBLIC
        pcb::common
        ${OpenCV_LIBS}
)

add_library(pcb::calibration ALIAS calibration)

