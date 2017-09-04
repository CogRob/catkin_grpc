cmake_minimum_required(VERSION 3.1.0)
include(ExternalProject)
project(grpc)

find_package(catkin REQUIRED)

set(GRPC_URL https://github.com/grpc/grpc.git)
set(GRPC_TAG "v1.4.x")

catkin_package(
  LIBRARIES
    ares
    boringssl
    gpr
    grpc
    grpc++
    grpc_cronet
    grpc++_cronet
    grpc++_error_details
    grpc_plugin_support
    grpc++_reflection
    grpc_unsecure
    grpc++_unsecure
  CFG_EXTRAS generate_proto.cmake
)

ExternalProject_Add(
  build_grpc
  PREFIX grpc
  GIT_REPOSITORY ${GRPC_URL}
  GIT_TAG ${GRPC_TAG}
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE 1
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND ""
)

set(GRPC_INCLUDE_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
set(GRPC_LIB_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set(GRPC_BIN_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY
    ${GRPC_INCLUDE_DESTINATION}/google
    ${GRPC_INCLUDE_DESTINATION}/grpc
    ${GRPC_INCLUDE_DESTINATION}/grpc++
  USE_SOURCE_PERMISSIONS
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(
  DIRECTORY ${GRPC_LIB_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
install(
  DIRECTORY ${GRPC_BIN_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

find_program(RSYNC rsync)
if(NOT RSYNC)
  message(SEND_ERROR "Cannot find rsync.")
endif(NOT RSYNC)

add_custom_command(
  TARGET build_grpc
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_INCLUDE_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_LIB_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_BIN_DESTINATION}
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${CMAKE_CURRENT_BINARY_DIR}/grpc/src/build_grpc/libs/opt/
      ${GRPC_LIB_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${CMAKE_CURRENT_BINARY_DIR}/grpc/src/build_grpc/bins/opt/
      ${GRPC_BIN_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --exclude="*"
      ${CMAKE_CURRENT_BINARY_DIR}/grpc/src/build_grpc/include/
      ${GRPC_INCLUDE_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --include="*.proto" --exclude="*"
      ${CMAKE_CURRENT_BINARY_DIR}/grpc/src/build_grpc/third_party/protobuf/src/
      ${GRPC_INCLUDE_DESTINATION}/
)
