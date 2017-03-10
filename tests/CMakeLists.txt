add_executable(test_defer test.cpp)
set_target_properties(test_defer PROPERTIES COMPILE_FLAGS "-std=c++1z -Wall -Wextra")
add_test(NAME test_defer COMMAND test_defer)

function(vir_add_run_target target)
   if("${CMAKE_GENERATOR}" MATCHES "Visual Studio")
      # do nothing. This just clutters the solution explorer
   else()
      add_custom_target(run_${target} ALL
         ${target} -v
         DEPENDS ${target}
         COMMENT "Execute ${target} test"
         VERBATIM
         )
   endif()
endfunction()
vir_add_run_target(test_defer)
