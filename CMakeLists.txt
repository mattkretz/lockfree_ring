project(lockfree_ring)
cmake_minimum_required(VERSION 2.6)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/virtest/vir/test.h")
   # first try to resolve the issue automatically
   message(STATUS "virtest missing: calling 'git submodule update'")
   execute_process(
      COMMAND git submodule update
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      )
   if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/virtest/vir/test.h")
      message(FATAL_ERROR "Unit tests enabled but virtest is missing. Call 'git submodule update' to clone the required unit test framework.")
   endif()
endif()

enable_testing()
add_subdirectory(tests)

install(FILES lockfree_ring.h DESTINATION include/vir/)
