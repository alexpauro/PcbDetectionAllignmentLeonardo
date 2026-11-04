add_library(board_alignment
    src/gerber_parser.cpp
    src/board_edge_detector.cpp
    src/board_alignment.cpp
)

target_include_directories(board_alignment
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(board_alignment
    PUBLIC
        ${OpenCV_LIBS}
)

add_library(pcb::board_alignment ALIAS board_alignment)

