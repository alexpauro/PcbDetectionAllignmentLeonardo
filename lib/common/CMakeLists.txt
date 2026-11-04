add_library(common STATIC
    src/common_utils.cpp
)

target_include_directories(common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(common
    PUBLIC
        ${OpenCV_LIBS}
)

add_library(pcb::common ALIAS common)

