function(
  rosidl_get_typesupport_target
  var
  generate_interfaces_target
  typesupport_name)
  if(NOT TARGET ${generate_interfaces_target})
    message(
      FATAL_ERROR
        "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?"
    )
  endif()

  set(output_target "${generate_interfaces_target}__${typesupport_name}")

  if(NOT TARGET ${output_target})
    # CMake if() evaluates strings ending in `-NOTFOUND` as false
    set(output_target "${output_target}-NOTFOUND")
  endif()

  set("${var}" "${output_target}" PARENT_SCOPE)
endfunction()
