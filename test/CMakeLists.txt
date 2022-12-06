cmake_minimum_required(VERSION 3.14)

project(vsgTests LANGUAGES CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)
include(../cmake/windows-set-path.cmake)

# ---- Dependencies ----

if(PROJECT_IS_TOP_LEVEL)
  find_package(vsg REQUIRED)
  enable_testing()
endif()

find_package(doctest REQUIRED)
find_package(immer REQUIRED)

# ---- Tests ----

add_executable(vsg_test source/vsg_test.cpp)
target_link_libraries(
    vsg_test PRIVATE
    vsg::vsg
    doctest::doctest
    immer::immer
)
target_compile_features(vsg_test PRIVATE cxx_std_20)

add_test(NAME vsg_test COMMAND vsg_test)

# ---- End-of-file commands ----

add_folders(Test)
