# Copyright 2019 Analog Devices, Inc.
#
# Licensed under the ADI BSD

cmake_minimum_required(VERSION 2.8.12)
project(fancontrold C)

set(BIN fancontrold)
set(SRC fancontrold.c)

include(GNUInstallDirs)

add_compile_options(-O2 -Wall -Wextra -Werror)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio.h)

include_directories(
	${LIBIIO_INCLUDE_DIRS})

add_executable(${BIN} ${SRC})
target_link_libraries(${BIN} ${LIBIIO_LIBRARIES})

install(TARGETS ${BIN}
	RUNTIME
	DESTINATION ${CMAKE_INSTALL_SBINDIR})
