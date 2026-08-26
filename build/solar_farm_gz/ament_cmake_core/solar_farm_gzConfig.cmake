# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_solar_farm_gz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED solar_farm_gz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(solar_farm_gz_FOUND FALSE)
  elseif(NOT solar_farm_gz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(solar_farm_gz_FOUND FALSE)
  endif()
  return()
endif()
set(_solar_farm_gz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT solar_farm_gz_FIND_QUIETLY)
  message(STATUS "Found solar_farm_gz: 0.1.0 (${solar_farm_gz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'solar_farm_gz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT solar_farm_gz_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(solar_farm_gz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${solar_farm_gz_DIR}/${_extra}")
endforeach()
