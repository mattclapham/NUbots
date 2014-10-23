if (NUROBOTX_LIBRARIES AND NUROBOTX_INCLUDE_DIRS)
  # in cache already
  set(NUROBOTX_FOUND TRUE)
else (NUROBOTX_LIBRARIES AND NUROBOTX_INCLUDE_DIRS)

  find_path(NUROBOTX_INCLUDE_DIR
    NAMES
      NURobotX
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(NUROBOTX_LIBRARY
    NAMES
      NURobotX
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(NUROBOTX_INCLUDE_DIRS
    ${NUROBOTX_INCLUDE_DIR}
  )

  if (NUROBOTX_LIBRARY)
    set(NUROBOTX_LIBRARIES
        ${NUROBOTX_LIBRARIES}
        ${NUROBOTX_LIBRARY}
    )
  endif (NUROBOTX_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(NURobotX DEFAULT_MSG NUROBOTX_LIBRARY NUROBOTX_INCLUDE_DIR)

  # show the NUROBOTX_INCLUDE_DIR and NUROBOTX_LIBRARY variables only in the advanced view
  mark_as_advanced(NUROBOTX_INCLUDE_DIRS FLYCAP_LIBRARIES)

endif (NUROBOTX_LIBRARIES AND NUROBOTX_INCLUDE_DIRS)
