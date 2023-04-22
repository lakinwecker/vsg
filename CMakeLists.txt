cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    vsg
    VERSION 0.1.0
    DESCRIPTION "A virtual scene-graph for unidirectional data flow UIs. Inspired by the virtual dom from snabbdom/React/Elm/Vue.js"
    HOMEPAGE_URL "https://github.com/lakinwecker/vsg"
    LANGUAGES NONE
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)
include(vsg_dependencies)
include(vsg_targets)

# ---- Dependencies ----
vsg_find_dependencies_thirdparty()

# ---- Declare library ----
vsg_declare_library()

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Examples ----

if(PROJECT_IS_TOP_LEVEL)
  option(BUILD_EXAMPLES "Build examples tree." "${vsg_DEVELOPER_MODE}")
  if(BUILD_EXAMPLES)
    add_subdirectory(example)
  endif()
endif()

# ---- Developer mode ----

if(NOT vsg_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of vsg"
  )
endif()

include(cmake/dev-mode.cmake)
