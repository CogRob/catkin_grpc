cmake_minimum_required(VERSION 3.8.2)
include(ExternalProject)
project(grpc)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS generate_proto.cmake
)

find_program(RSYNC rsync)
if(NOT RSYNC)
  message(SEND_ERROR "Cannot find rsync.")
endif(NOT RSYNC)

set(GRPC_BUILD_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/grpc_build")
set(GRPC_BUILD_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/grpc_install)
ExternalProject_Add(
  build_grpc
  PREFIX grpc

  # Copy grpc submodule to build directory to use artifact objects in the source directory.
  # BUILD_IN_SOURCE 1 allows ExternalProject_Add() to generates the artifact objects in the
  # source directory.
  DOWNLOAD_COMMAND
  ${RSYNC}
  --prune-empty-dirs --archive
  ${CMAKE_SOURCE_DIR}/grpc/ ${GRPC_BUILD_SOURCE_DIR}/

  SOURCE_DIR ${GRPC_BUILD_SOURCE_DIR}

  CMAKE_ARGS
  -DBUILD_SHARED_LIBS=ON
  -DCARES_SHARED=ON
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_INSTALL_PREFIX=${GRPC_BUILD_INSTALL_DIR}
  -DgRPC_BUILD_TESTS=OFF
  -DgRPC_INSTALL=ON
  -DgRPC_SSL_PROVIDER=package
  -DgRPC_ZLIB_PROVIDER=package
  -Dprotobuf_BUILD_LIBPROTOC=ON
  -Dprotobuf_BUILD_PROTOC_BINARIES=ON
  -Dprotobuf_BUILD_SHARED_LIBS=OFF
  -Dprotobuf_BUILD_TESTS=OFF
  -Dprotobuf_WITH_ZLIB=ON
  BUILD_IN_SOURCE 1
)

set(GRPC_INCLUDE_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc)
set(GRPC_LIB_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set(GRPC_BIN_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
set(GRPC_SHARE_DESTINATION
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})

add_custom_command(
  TARGET build_grpc
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_INCLUDE_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_LIB_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_BIN_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_SHARE_DESTINATION}
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --exclude="*.cmake" --exclude="libproto*"
      ${GRPC_BUILD_INSTALL_DIR}/lib/
      ${GRPC_LIB_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="libproto*" --exclude="*"
      ${GRPC_BUILD_INSTALL_DIR}/lib/
      ${GRPC_LIB_DESTINATION}/protobuf
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${GRPC_BUILD_INSTALL_DIR}/bin/
      ${GRPC_BIN_DESTINATION}/
  COMMAND
    ${CMAKE_COMMAND} -E make_directory ${GRPC_INCLUDE_DESTINATION}/src/proto
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.proto" --exclude="*"
      ${GRPC_BUILD_SOURCE_DIR}/src/proto/
      ${GRPC_INCLUDE_DESTINATION}/src/proto/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --exclude="*"
      ${GRPC_BUILD_SOURCE_DIR}/gens/src/proto/
      ${GRPC_INCLUDE_DESTINATION}/src/proto/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --exclude="*"
      ${GRPC_BUILD_INSTALL_DIR}/include/
      ${GRPC_INCLUDE_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --include="*.inc" --include="*.proto" --exclude="*"
      ${GRPC_BUILD_SOURCE_DIR}/third_party/protobuf/src/
      ${GRPC_INCLUDE_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${GRPC_BUILD_INSTALL_DIR}/share/
      ${GRPC_SHARE_DESTINATION}/
)

install(
  DIRECTORY ${GRPC_INCLUDE_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc)
install(
  DIRECTORY ${GRPC_LIB_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
install(
  DIRECTORY ${GRPC_BIN_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
install(
  DIRECTORY ${GRPC_SHARE_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
