#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cam2image_vm2ros::cam2image_vm2ros" for configuration ""
set_property(TARGET cam2image_vm2ros::cam2image_vm2ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(cam2image_vm2ros::cam2image_vm2ros PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcam2image_vm2ros.so"
  IMPORTED_SONAME_NOCONFIG "libcam2image_vm2ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS cam2image_vm2ros::cam2image_vm2ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_cam2image_vm2ros::cam2image_vm2ros "${_IMPORT_PREFIX}/lib/libcam2image_vm2ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
