cmake_minimum_required(VERSION 3.20)
project(xbot2_py_bridge VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)

find_package(xbot2 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_xbot2_device(py_bridge_hal src/py_bridge_hal.cpp)
target_link_libraries(py_bridge_hal PRIVATE nlohmann_json::nlohmann_json)

install(TARGETS py_bridge_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
