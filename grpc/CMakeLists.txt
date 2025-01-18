cmake_minimum_required(VERSION 3.8.2)
include(ExternalProject)
project(grpc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS generate_proto.cmake
)

find_program(RSYNC rsync)
if(NOT RSYNC)
  message(SEND_ERROR "Cannot find rsync.")
endif(NOT RSYNC)

set(GRPC_BUILD_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/grpc_install)
ExternalProject_Add(
  build_grpc
  PREFIX grpc
  GIT_REPOSITORY "https://github.com/grpc/grpc.git"
  GIT_TAG v1.34.1
  GIT_CONFIG submodule.recurse=1 submodule.fetchJobs=10
  GIT_SHALLOW 1
  SOURCE_DIR grpc_build
  CMAKE_ARGS
  -DBUILD_SHARED_LIBS=ON
  -DBUILD_TESTING=OFF
  -DCARES_SHARED=ON
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_INSTALL_PREFIX=${GRPC_BUILD_INSTALL_DIR}
  -DgRPC_BUILD_TESTS=OFF
  -DgRPC_INSTALL=ON
  -Dprotobuf_BUILD_LIBPROTOC=ON
  -Dprotobuf_BUILD_PROTOC_BINARIES=ON
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
      ${GRPC_BUILD_INSTALL_DIR}/lib/
      ${GRPC_LIB_DESTINATION}/
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
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/src/proto/
      ${GRPC_INCLUDE_DESTINATION}/src/proto/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --exclude="*"
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/gens/src/proto/
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
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/third_party/protobuf/src/
      ${GRPC_INCLUDE_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${GRPC_BUILD_INSTALL_DIR}/share/
      ${GRPC_SHARE_DESTINATION}/
)

install(
  DIRECTORY
    ${GRPC_INCLUDE_DESTINATION}/google
    ${GRPC_INCLUDE_DESTINATION}/grpc
    ${GRPC_INCLUDE_DESTINATION}/grpc++
    ${GRPC_INCLUDE_DESTINATION}/grpcpp
    ${GRPC_INCLUDE_DESTINATION}/src
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
