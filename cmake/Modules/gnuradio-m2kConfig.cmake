find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_M2K gnuradio-m2k)

FIND_PATH(
    GR_M2K_INCLUDE_DIRS
    NAMES gnuradio/m2k/api.h
    HINTS $ENV{M2K_DIR}/include
        ${PC_M2K_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_M2K_LIBRARIES
    NAMES gnuradio-m2k
    HINTS $ENV{M2K_DIR}/lib
        ${PC_M2K_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-m2kTargets.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_M2K DEFAULT_MSG GR_M2K_LIBRARIES GR_M2K_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_M2K_LIBRARIES GR_M2K_INCLUDE_DIRS)
