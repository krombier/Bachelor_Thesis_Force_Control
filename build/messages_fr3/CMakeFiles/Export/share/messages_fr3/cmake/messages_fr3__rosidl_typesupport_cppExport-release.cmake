#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "messages_fr3::messages_fr3__rosidl_typesupport_cpp" for configuration "Release"
set_property(TARGET messages_fr3::messages_fr3__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(messages_fr3::messages_fr3__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmessages_fr3__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_RELEASE "libmessages_fr3__rosidl_typesupport_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS messages_fr3::messages_fr3__rosidl_typesupport_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_messages_fr3::messages_fr3__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libmessages_fr3__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
