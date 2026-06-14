add_custom_target(install_python_pkg ALL
    COMMAND python -m pip show xbot2_py_bridge || python -m pip install -e ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Installing Python package in editable mode"
    DEPENDS py_bridge_hal
)

add_test(
    NAME pytest
    COMMAND python -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/tests -v
)

set_tests_properties(pytest PROPERTIES
    DEPENDS install_python_pkg
)