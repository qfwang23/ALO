if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

function(find_external_dependecy PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)
  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
  if(${${USE_FROM_SYSTEM_OPTION}})
    find_package(${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT ${${USE_FROM_SYSTEM_OPTION}} OR NOT TARGET ${TARGET_NAME})
    set(${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
    include(${INCLUDED_CMAKE_PATH})
  endif()
endfunction()

find_external_dependecy("Eigen3" "Eigen3::Eigen" "${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")
find_external_dependecy("Sophus" "Sophus::Sophus" "${CMAKE_CURRENT_LIST_DIR}/sophus/sophus.cmake")
find_external_dependecy("TBB" "TBB::tbb" "${CMAKE_CURRENT_LIST_DIR}/tbb/tbb.cmake")
find_external_dependecy("tsl-robin-map" "tsl::robin_map" "${CMAKE_CURRENT_LIST_DIR}/tsl_robin/tsl_robin.cmake")
