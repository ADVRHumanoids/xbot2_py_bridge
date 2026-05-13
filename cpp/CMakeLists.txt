cmake_minimum_required(VERSION 3.20)
project(xbot2_py_bridge VERSION 1.0.0)

find_package(xbot2 REQUIRED)

add_xbot2_device(py_bridge_hal src/py_bridge_hal.cpp)

install(TARGETS py_bridge_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
