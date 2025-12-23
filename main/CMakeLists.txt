idf_component_register(SRCS main.cpp esp32_serial_transport.c INCLUDE_DIRS "")

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--allow-multiple-definition")