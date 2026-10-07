
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

mcux_add_configuration(
    CC "-DSDK_DEBUGCONSOLE=1"
    CX "-DSDK_DEBUGCONSOLE=1"
)


mcux_add_source(
    SOURCES frdmmcxa156/board.c
            frdmmcxa156/board.h
)

mcux_add_include(
    INCLUDES frdmmcxa156
)

mcux_add_source(
    SOURCES board/clock_config.c
            board/clock_config.h
)

mcux_add_include(
    INCLUDES board
)

mcux_add_source(
    SOURCES board/pin_mux.c
            board/pin_mux.h
)

mcux_add_include(
    INCLUDES board
)

mcux_add_source(
    SOURCES led_blinky/app.h
            led_blinky/hardware_init.c
)

mcux_add_include(
    INCLUDES led_blinky
)
