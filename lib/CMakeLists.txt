# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR} ${IIO_INCLUDE_DIRS} ${LIBM2K_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND m2k_sources
        analog_in_converter_impl.cc
        analog_in_source_impl.cc
    )

set(m2k_sources "${m2k_sources}" PARENT_SCOPE)
if (NOT m2k_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif (NOT m2k_sources)

add_library(gnuradio-m2k SHARED ${m2k_sources})
target_link_libraries(gnuradio-m2k
        gnuradio::gnuradio-runtime
        gnuradio::gnuradio-blocks
        ${IIO_LIBRARIES}
        ${M2K_LIBRARIES})

target_include_directories(gnuradio-m2k
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
        PUBLIC $<INSTALL_INTERFACE:include>
        )

set_target_properties(gnuradio-m2k PROPERTIES
        VERSION ${GR_M2K_VERSION}
        SOVERSION ${GR_M2K_VERSION_MAJOR}
        DEFINE_SYMBOL "gnuradio_m2k_EXPORTS"
        )

if (APPLE)
    set_target_properties(gnuradio-m2k PROPERTIES
            INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
            )
endif (APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-m2k)
