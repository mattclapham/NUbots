# - Try to find DC1394
# Once done this will define
#
#  DC1394_FOUND - system has DC1394
#  DC1394_INCLUDE_DIRS - the DC1394 include directory
#  DC1394_LIBRARIES - Link these to use DC1394
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (DC1394_LIBRARIES AND DC1394_INCLUDE_DIRS)
  # in cache already
  set(DC1394_FOUND TRUE)
else (DC1394_LIBRARIES AND DC1394_INCLUDE_DIRS)

  find_path(DC1394_INCLUDE_DIR
    NAMES
      dc1394/control.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(DC1394_LIBRARY
    NAMES
      dc1394
    PATHS
      /usr/lib
      /usr/lib/i386-linux-gnu
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(DC1394_INCLUDE_DIRS
    ${DC1394_INCLUDE_DIR}
  )

  if (DC1394_LIBRARY)
    set(DC1394_LIBRARIES
        ${DC1394_LIBRARIES}
        ${DC1394_LIBRARY}
    )
  endif (DC1394_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(DC1394 DEFAULT_MSG DC1394_LIBRARIES DC1394_INCLUDE_DIRS)

  # show the DC1394_INCLUDE_DIRS and DC1394_LIBRARIES variables only in the advanced view
  mark_as_advanced(DC1394_INCLUDE_DIRS DC1394_LIBRARIES)

endif (DC1394_LIBRARIES AND DC1394_INCLUDE_DIRS)

