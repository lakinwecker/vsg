cmake_minimum_required(VERSION 3.14)

project(vsgExamples CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

if(PROJECT_IS_TOP_LEVEL)
  find_package(vsg REQUIRED)
endif()

add_custom_target(run-examples)

function(add_example NAME)
  find_package(doctest REQUIRED)
  find_package(immer REQUIRED)

  add_executable("${NAME}" "${NAME}.cpp")
  target_link_libraries(
    "${NAME}" PRIVATE
    vsg::vsg
    doctest::doctest
    immer::immer
  )
  target_compile_features("${NAME}" PRIVATE cxx_std_20)
  add_custom_target("run_${NAME}" COMMAND "${NAME}" VERBATIM)
  add_dependencies("run_${NAME}" "${NAME}")
  add_dependencies(run-examples "run_${NAME}")
endfunction()

add_example(empty_example)

add_folders(Example)
