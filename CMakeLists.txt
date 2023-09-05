# Copyright 2019 Analog Devices, Inc.
#
# Licensed under the ADI BSD

cmake_minimum_required(VERSION 3.10)
project(fancontrold C)

set(BIN fancontrold)
set(SRC fancontrold.c)

include(GNUInstallDirs)

add_compile_options(-O2 -Wall -Wextra -Werror)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio/iio.h)

add_executable(${BIN} ${SRC})
target_include_directories(${BIN} PRIVATE ${LIBIIO_INCLUDE_DIRS})
target_link_libraries(${BIN} ${LIBIIO_LIBRARIES})

install(TARGETS ${BIN}
	RUNTIME
	DESTINATION ${CMAKE_INSTALL_SBINDIR})

add_subdirectory(init)
