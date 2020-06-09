cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(stoch3_raisim)

find_package(PythonLibs 2.7)
find_package(raisimOgre CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories (${EIGEN3_INCLUDE_DIRS})
find_package(towr 1.2 REQUIRED)


function(create_executable app_name file_name)
  add_executable(${app_name} ${file_name})

 
  target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
  target_include_directories(${app_name} PUBLIC ${PYTHON_INCLUDE_DIRS})

  target_link_libraries(${app_name} PUBLIC ${PYTHON_LIBRARIES})
  target_link_libraries(${app_name} PUBLIC raisim::raisimOgre)
  
  target_compile_definitions(${app_name} PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc/")

endfunction()


function(create_sharedlib lib_name file_name)
  

  ADD_LIBRARY(${lib_name} SHARED ${file_name})

  target_include_directories(${lib_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
  target_include_directories(${lib_name} PUBLIC ${PYTHON_INCLUDE_DIRS})

  target_link_libraries(${lib_name} PUBLIC ${PYTHON_LIBRARIES})
  target_link_libraries(${lib_name} PUBLIC raisim::raisimOgre)
  

  target_compile_definitions(${lib_name} PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc/")

endfunction()

create_sharedlib(stoch3_raisim ./lib_src/stoch3_raisim_lib.cpp)
create_executable(stoch3_trot ./lib_src/stoch3_pd_control.cpp)


