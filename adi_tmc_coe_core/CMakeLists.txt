cmake_minimum_required(VERSION 3.8)
project(adi_tmc_coe_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(adi_tmc_coe_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

###################################
## External Project Dependencies ##
###################################

message(STATUS " CMAKE_CURRENT_SOURCE_DIR is: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS " CMAKE_BINARY_DIR is: " ${CMAKE_BINARY_DIR})
message(STATUS " CMAKE_INSTALL_PREFIX is: " ${CMAKE_INSTALL_PREFIX})
set(CURRENT_CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

include(ExternalProject)
ExternalProject_Add(SOEM
  #  SOURCE_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/SOEM
  #  STAMP_DIR           ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/SOEM
  #  TMP_DIR             ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/SOEM
  #  CONFIGURE_COMMAND   bash -c "mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/SOEM"
   GIT_REPOSITORY      https://github.com/OpenEtherCATsociety/SOEM.git
   GIT_TAG             v1.4.0
  #  BINARY_DIR          ${CMAKE_BINARY_DIR}/dependencies/SOEM
   INSTALL_DIR         ${CURRENT_CMAKE_INSTALL_PREFIX}
   BUILD_COMMAND       cmake -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_POSITION_INDEPENDENT_CODE=ON <SOURCE_DIR>
   INSTALL_COMMAND     make install
   TEST_COMMAND        echo ">>> SOEM: " COMMAND echo <SOURCE_DIR> COMMAND echo <BINARY_DIR>
   LOG_DOWNLOAD        1
   LOG_UPDATE          1
   LOG_PATCH           1
   LOG_CONFIGURE       1
   LOG_BUILD           1
   LOG_INSTALL         1
   LOG_TEST            1
 )

 ExternalProject_Get_Property(SOEM SOURCE_DIR)
 message(STATUS " SOEM's source_dir is: " ${SOURCE_DIR})

 ExternalProject_Get_Property(SOEM BINARY_DIR)
 message(STATUS " SOEM's binary_dir is: " ${BINARY_DIR})

 ExternalProject_Get_Property(SOEM INSTALL_DIR)
 message(STATUS " SOEM's install_dir is: " ${INSTALL_DIR})

# add dependencies on SOEM
set(SOEM_INCLUDE ${INSTALL_DIR}/include/soem)
set (SOEM_BUILD ${INSTALL_DIR}/lib/libsoem.a)

add_library(soem_target SHARED IMPORTED)
set_target_properties(soem_target PROPERTIES IMPORTED_LOCATION ${SOEM_BUILD})
add_dependencies(soem_target SOEM)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating ros_ethercat_loop RUNPATH to dependencies, in contrast to RPATH
SET(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
############################################

# Install include directory
include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

############################################

add_library(${PROJECT_NAME}_tmc_coe_interpreter SHARED
  src/tmc_coe_interpreter.cpp)

target_link_libraries(${PROJECT_NAME}_tmc_coe_interpreter ${SOEM_BUILD} soem_target)
target_include_directories(${PROJECT_NAME}_tmc_coe_interpreter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${SOEM_INCLUDE}
)

ament_target_dependencies(${PROJECT_NAME}_tmc_coe_interpreter rclcpp)
ament_export_targets(${PROJECT_NAME}_tmc_coe_interpreterTargets HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}_tmc_coe_interpreter
  EXPORT ${PROJECT_NAME}_tmc_coe_interpreterTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmc_coe_motor SHARED
  src/tmc_coe_motor.cpp)

target_link_libraries(tmc_coe_motor ${PROJECT_NAME}_tmc_coe_interpreter)
target_include_directories(tmc_coe_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmc_coe_motor rclcpp geometry_msgs std_msgs adi_tmc_coe_interfaces)
ament_export_targets(tmc_coe_motorTargets HAS_LIBRARY_TARGET)

install(
  TARGETS tmc_coe_motor
  EXPORT tmc_coe_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmc_coe_bldc_motor SHARED
  src/tmc_coe_bldc_motor.cpp)

target_link_libraries(tmc_coe_bldc_motor ${PROJECT_NAME}_tmc_coe_interpreter tmc_coe_motor)
target_include_directories(tmc_coe_bldc_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmc_coe_bldc_motor rclcpp adi_tmc_coe_interfaces)
ament_export_targets(tmc_coe_bldc_motorTargets HAS_LIBRARY_TARGET)

install(
  TARGETS tmc_coe_bldc_motor
  EXPORT tmc_coe_bldc_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmc_coe_stepper_motor SHARED
  src/tmc_coe_stepper_motor.cpp)

target_link_libraries(tmc_coe_stepper_motor ${PROJECT_NAME}_tmc_coe_interpreter tmc_coe_motor)
target_include_directories(tmc_coe_stepper_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmc_coe_stepper_motor rclcpp adi_tmc_coe_interfaces)
ament_export_targets(tmc_coe_stepper_motorTargets HAS_LIBRARY_TARGET)

install(
  TARGETS tmc_coe_stepper_motor
  EXPORT tmc_coe_stepper_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmc_coe_ros2_lib SHARED
  src/tmc_coe_ros2.cpp)

target_link_libraries(tmc_coe_ros2_lib ${PROJECT_NAME}_tmc_coe_interpreter tmc_coe_motor tmc_coe_bldc_motor tmc_coe_stepper_motor)
target_include_directories(tmc_coe_ros2_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmc_coe_ros2_lib rclcpp adi_tmc_coe_interfaces)
ament_export_targets(tmc_coe_ros2_libTargets HAS_LIBRARY_TARGET)

install(
  TARGETS tmc_coe_ros2_lib
  EXPORT tmc_coe_ros2_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################


add_executable(tmc_coe_ros2_node src/tmc_coe_ros2_node.cpp)
target_include_directories(tmc_coe_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tmc_coe_ros2_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(tmc_coe_ros2_node tmc_coe_ros2_lib)

ament_target_dependencies(tmc_coe_ros2_node rclcpp)

install(TARGETS tmc_coe_ros2_node
  DESTINATION lib/${PROJECT_NAME})

############################################

############################################

add_executable(adi_ethercat_grant_node src/adi_ethercat_grant_node.cpp)
target_include_directories(adi_ethercat_grant_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(adi_ethercat_grant_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(adi_ethercat_grant_node cap)

ament_target_dependencies(adi_ethercat_grant_node rclcpp)

install(TARGETS adi_ethercat_grant_node
  DESTINATION lib/${PROJECT_NAME})

############################################

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

############################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
