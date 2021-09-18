cmake_minimum_required(VERSION 3.0)
project(
    "Localisation"
    VERSION 0.0.0
    DESCRIPTION "Localisation methods"
    LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/SFML/cmake/Modules")
find_package(SFML 2 REQUIRED graphics window system)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC
    src/geometry.cpp
    src/main.cpp
    src/render_utils.cpp
    src/sensor.cpp
    src/terrain.cpp
)

add_executable(localisation ${SRC})

target_include_directories(localisation
    PRIVATE include
)

target_link_libraries(localisation
    PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
        Eigen3::Eigen
)
