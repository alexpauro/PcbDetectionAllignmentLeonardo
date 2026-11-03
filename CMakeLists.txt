cmake_minimum_required(VERSION 3.16)
project(PcbDetectionAllignmentLeonardo VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warning di compilazione
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra)
endif()

# Ricerca dipendenze esterne
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV versione trovata: ${OpenCV_VERSION}")

# Sottodirectory
add_subdirectory(lib)
add_subdirectory(app)

