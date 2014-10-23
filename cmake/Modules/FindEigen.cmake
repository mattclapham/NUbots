if (EIGEN_INCLUDE_DIRS)
  # in cache already
  set(EIGEN_FOUND TRUE)
else (EIGEN_INCLUDE_DIRS)

  find_path(EIGEN_INCLUDE_DIR
    NAMES
      eigen3/Eigen/Core
    PATHS
      /usr/include/
      /usr/local/include/
      /opt/local/include/
      /sw/include/
  )

  set(EIGEN_INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIRS)

  # show the EIGEN_INCLUDE_DIRS variables only in the advanced view
  mark_as_advanced(EIGEN_INCLUDE_DIRS)

endif (EIGEN_INCLUDE_DIRS)
