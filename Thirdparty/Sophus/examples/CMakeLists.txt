cmake_minimum_required(VERSION 3.24)

project(SophusExample)

set(_sophus_version_requirement "${SOPHUS_VERSION}")
if(NOT _sophus_version_requirement)
  set(_sophus_version_file "${CMAKE_CURRENT_LIST_DIR}/../SOPHUS_VERSION")
  if(EXISTS "${_sophus_version_file}")
    file(READ "${_sophus_version_file}" _sophus_version_contents)
    string(STRIP "${_sophus_version_contents}" _sophus_version_requirement)
  else()
    set(_sophus_version_requirement "1.0.0")
  endif()
endif()
unset(_sophus_version_contents)
unset(_sophus_version_file)
find_package(Sophus ${_sophus_version_requirement} REQUIRED)
set(CMAKE_CXX_STANDARD 17)

# Release by default Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Tests to run
set(EXAMPLE_SOURCES hello_so3 ensure_example)

option(SOPHUS_ENABLE_ENSURE_HANDLER "Enable the custem ensure handler." OFF)
if(SOPHUS_ENABLE_ENSURE_HANDLER)
  add_library(custom_ensure_handler custom_ensure_handler.cpp)
  target_link_libraries(custom_ensure_handler PRIVATE Sophus::Sophus)
endif()

foreach(example_src ${EXAMPLE_SOURCES})
  add_executable(${example_src} ${example_src}.cpp)
  if(SOPHUS_ENABLE_ENSURE_HANDLER)
    target_compile_definitions(${example_src}
                               PRIVATE SOPHUS_ENABLE_ENSURE_HANDLER=1)
    target_link_libraries(${example_src} Sophus::Sophus custom_ensure_handler)
  else()
    target_link_libraries(${example_src} Sophus::Sophus)
  endif()
endforeach(example_src)
