# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:robo24_interfaces__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:robo24_interfaces__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:robo24_interfaces__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:robo24_interfaces__rosidl_typesupport_c;__rosidl_generator_cpp:robo24_interfaces__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:robo24_interfaces__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:robo24_interfaces__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:robo24_interfaces__rosidl_typesupport_cpp;__rosidl_generator_py:robo24_interfaces__rosidl_generator_py")

# populate robo24_interfaces_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "robo24_interfaces::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'robo24_interfaces' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND robo24_interfaces_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
