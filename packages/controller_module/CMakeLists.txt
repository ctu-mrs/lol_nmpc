cmake_minimum_required(VERSION 3.1.2)
project(controller_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  mavros_msgs
  sensor_msgs
  std_srvs
  nodelet
  mrs_uav_managers
  mrs_lib
  dynamic_reconfigure
  )


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})


file(REMOVE_RECURSE src/nmpc_acados/c_generated_code/)
execute_process(COMMAND python src/nmpc_acados/main_build.py 
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

execute_process(COMMAND bash scripts/githash.sh 
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE DOT_O_FILES src/nmpc_acados/c_generated_code/*.o)
file (REMOVE
    ${DOT_O_FILES}
)
file(GLOB_RECURSE DOT_SO_FILES src/nmpc_acados/c_generated_code/*.so)
file (REMOVE
  ${DOT_SO_FILES}
)

generate_dynamic_reconfigure_options(
  cfg/controller_params.cfg
  )

add_message_files(
  FILES
  NmpcControlOutput.msg
  DroneStateMsg.msg
  Telemetry.msg
  AcadosParamsMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  controller_module
)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet mrs_msgs mavros_msgs mrs_uav_managers mrs_lib std_srvs dynamic_reconfigure
  INCLUDE_DIRS src/nmpc_acados/hpp
  DEPENDS Eigen
  LIBRARIES ControllerModule
  )

set(ACADOS_INCLUDE_DIR $ENV{ACADOS_SOURCE_DIR}/include)
set(ACADOS_LIB_DIR $ENV{ACADOS_SOURCE_DIR}/lib)
FILE(GLOB model_src src/nmpc_acados/c_generated_code/quadrotor_ode_model/*.c)
FILE(GLOB ocp_src src/nmpc_acados/c_generated_code/quadrotor_ode_cost/*.c)
FILE(GLOB contraints_src src/nmpc_acados/c_generated_code/quadrotor_ode_constraints/*.c)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(yaml-cpp REQUIRED)
find_library(ACADOS ${ACADOS_LIB_DIR}/libacados.so)



include_directories(
    ${ACADOS_INCLUDE_DIR}
  hpp
  src/nmpc_acados/hpp
  src/propulsion_module/include
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include

  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# wrapper
add_library(ModelObj ${model_src})
add_dependencies(ModelObj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ModelObj
  ${catkin_LIBRARIES}
)

add_library(OcpObj ${ocp_src})
add_dependencies(OcpObj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OcpObj
  ${catkin_LIBRARIES}
)

# add_library(OcpConst ${contraints_src})
# add_dependencies(OcpConst ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(OcpConst
#   ${catkin_LIBRARIES}
# )

add_library(AcadosSim src/nmpc_acados/c_generated_code/acados_sim_solver_quadrotor_ode.c)
add_dependencies(AcadosSim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AcadosSim
  ${catkin_LIBRARIES}
)


add_library(TargetExample src/nmpc_acados/c_generated_code/main_quadrotor_ode.c src/nmpc_acados/c_generated_code/acados_solver_quadrotor_ode.c)
add_dependencies(TargetExample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TargetExample
OcpObj
ModelObj
# OcpConst
  ${catkin_LIBRARIES}
)
add_library(TargetExampleSim src/nmpc_acados/c_generated_code/main_sim_quadrotor_ode.c)
add_dependencies(TargetExampleSim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TargetExampleSim
  ${catkin_LIBRARIES}
)

add_library(ControllerModule
  src/controller_module.cpp
  src/nmpc_acados/cpp/drone.cpp
  src/nmpc_acados/cpp/acados_wrapper.cpp
  )

add_dependencies(ControllerModule
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControllerModule
 ${ACADOS_LIB_DIR}/libacados.so
 # yaml-cpp 
 ModelObj
 OcpObj
#  OcpConst
 AcadosSim
 TargetExample
 TargetExampleSim
  ${catkin_LIBRARIES}
  )

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

