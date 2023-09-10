if(NOT FTD3XX_FOUND)

    find_package(PkgConfig)
    pkg_check_modules (FTD3XX_PKG libftd3xx)
    set(FTD3XX_DEFINITIONS ${PC_FTD3XX_CFLAGS_OTHER})

    find_path(FTD3XX_INCLUDE_DIR
                NAMES ftd3xx.h
                HINTS ${FTD3XX_PKG_INCLUDE_DIRS} $ENV{FTD3XX_DIR}/include
                PATHS /usr/local/include /usr/include /opt/include /opt/local/include)

    find_library(FTD3XX_LIBRARY
            NAMES ftd3xx
            HINTS ${FTD3XX_PKG_LIBRARY_DIRS} $ENV{FTD3XX_DIR}/include
            PATHS /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

    set(FTD3XX_LIBRARIES ${FTD3XX_LIBRARY} )
    set(FTD3XX_INCLUDE_DIRS ${FTD3XX_INCLUDE_DIR} )

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(FTD3XX  DEFAULT_MSG
            FTD3XX_LIBRARY FTD3XX_INCLUDE_DIR)

    mark_as_advanced(FTD3XX_INCLUDE_DIR FTD3XX_LIBRARY)

endif(NOT FTD3XX_FOUND)

