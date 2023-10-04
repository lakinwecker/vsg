cmake_minimum_required(VERSION 3.14)

project(vsgTests LANGUAGES CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)
include(../cmake/windows-set-path.cmake)
include(vsg_dependencies)

# ---- Dependencies ----

if(PROJECT_IS_TOP_LEVEL)
  find_package(vsg REQUIRED)
  enable_testing()
endif()

vsg_find_dependencies_thirdparty()
vsg_dependency_targets()

# ---- Tests ----
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_link_options(-Wl,--disable-new-dtags)
endif()

add_executable(vsg_test source/vsg_test.cpp)
target_link_libraries(
    vsg_test PRIVATE
    vsg::vsg
    ${VSG_DEPENDENCY_TARGETS}
)
target_compile_features(vsg_test PRIVATE cxx_std_20)

# ---- End-of-file commands ----
add_folders(Test)
add_test(NAME vsg_test COMMAND vsg_test)
