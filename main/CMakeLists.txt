<<<<<<< HEAD
idf_component_register(SRCS main.cpp esp32_serial_transport.c INCLUDE_DIRS "")

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--allow-multiple-definition")
=======
idf_component_register(SRCS "main.cpp" "balancing-robot.c" esp32_serial_transport.c
                    INCLUDE_DIRS "")
>>>>>>> 6062975622c3767fd9c041fb4b267d3b85742423
