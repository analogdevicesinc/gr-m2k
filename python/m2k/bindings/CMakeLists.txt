# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT m2k_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT m2k_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################
list(APPEND m2k_python_files
    analog_in_converter_python.cc
    adf4350_sink_python.cc
    analog_out_converter_python.cc
    analog_out_sink_python.cc
    digital_in_source_python.cc
    mixed_signal_source_python.cc
    analog_in_source_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(m2k
   ../../..
   gr::m2k
   "${m2k_python_files}")

# copy in bindings .so file for use in QA test module
add_custom_command(TARGET m2k_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:m2k_python>
    ${CMAKE_BINARY_DIR}/test_modules/gnuradio/m2k/
)

install(TARGETS m2k_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/m2k COMPONENT pythonapi)
