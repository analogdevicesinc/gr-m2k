INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_M2K m2k)

FIND_PATH(
    M2K_INCLUDE_DIRS
    NAMES m2k/api.h
    HINTS $ENV{M2K_DIR}/include
        ${PC_M2K_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    M2K_LIBRARIES
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

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(M2K DEFAULT_MSG M2K_LIBRARIES M2K_INCLUDE_DIRS)
MARK_AS_ADVANCED(M2K_LIBRARIES M2K_INCLUDE_DIRS)
