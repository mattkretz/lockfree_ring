set(all "ALL")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/virtest")
if(DEFINED ENV{APPVEYOR})
   include_directories("C:/Libraries/boost")
   set(all)
endif()
add_executable(unittests test.cpp)
if(MSVC)
   #set_target_properties(unittests PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall -Wextra")
else()
   set_target_properties(unittests PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall -Wextra")
endif()
add_test(NAME unittests COMMAND unittests)

function(vir_add_run_target target)
   if("${CMAKE_GENERATOR}" MATCHES "Visual Studio")
      # do nothing. This just clutters the solution explorer
   else()
      add_custom_target(run_${target} ${all}
         ${target} -v
         DEPENDS ${target}
         COMMENT "Execute ${target} test"
         VERBATIM
         )
   endif()
endfunction()
vir_add_run_target(unittests)
