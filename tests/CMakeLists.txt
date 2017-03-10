if(DEFINED ENV{APPVEYOR})
   include_directories("C:/Libraries/boost")
endif()
add_executable(test_defer test.cpp)
if(MSVC)
   #set_target_properties(test_defer PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall -Wextra")
else()
   set_target_properties(test_defer PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall -Wextra")
endif()
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
