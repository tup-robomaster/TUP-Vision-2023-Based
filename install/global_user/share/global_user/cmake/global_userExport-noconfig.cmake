#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "global_user::global_user" for configuration ""
set_property(TARGET global_user::global_user APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(global_user::global_user PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libglobal_user.so"
  IMPORTED_SONAME_NOCONFIG "libglobal_user.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS global_user::global_user )
list(APPEND _IMPORT_CHECK_FILES_FOR_global_user::global_user "${_IMPORT_PREFIX}/lib/libglobal_user.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
