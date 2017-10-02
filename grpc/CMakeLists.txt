cmake_minimum_required(VERSION 2.8.12)
include(ExternalProject)
project(grpc)

find_package(catkin REQUIRED)

set(GRPC_TAG 8569dedd9b015c5159afdf157a2c6c6d7082a5c9)
set(GRPC_URL https://github.com/grpc/grpc/archive/${GRPC_TAG}.tar.gz)

set(C_ARES_TAG 7691f773af79bf75a62d1863fd0f13ebf9dc51b1)
set(C_ARES_URL https://github.com/c-ares/c-ares/archive/${C_ARES_TAG}.tar.gz)

set(BORINGSSL_TAG be2ee342d3781ddb954f91f8a7e660c6f59e87e5)
set(BORINGSSL_URL https://github.com/google/boringssl/archive/${BORINGSSL_TAG}.tar.gz)

set(BENCHMARK_TAG 44c25c892a6229b20db7cd9dc05584ea865896de)
set(BENCHMARK_URL https://github.com/google/benchmark/archive/${BENCHMARK_TAG}.tar.gz)

set(GFLAGS_TAG 30dbc81fb5ffdc98ea9b14b1918bfe4e8779b26e)
set(GFLAGS_URL https://github.com/gflags/gflags/archive/${GFLAGS_TAG}.tar.gz)

set(GOOGLETEST_TAG ec44c6c1675c25b9827aacd08c02433cccde7780)
set(GOOGLETEST_URL https://github.com/google/googletest/archive/${GOOGLETEST_TAG}.tar.gz)

set(PROTOBUF_TAG 80a37e0782d2d702d52234b62dd4b9ec74fd2c95)
set(PROTOBUF_URL https://github.com/google/protobuf/archive/${PROTOBUF_TAG}.tar.gz)

set(ZLIB_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f)
set(ZLIB_URL https://github.com/madler/zlib/archive/${ZLIB_TAG}.tar.gz)

catkin_package(
  CFG_EXTRAS generate_proto.cmake
)

find_program(RSYNC rsync)
if(NOT RSYNC)
  message(SEND_ERROR "Cannot find rsync.")
endif(NOT RSYNC)

ExternalProject_Add(get_c_ares PREFIX grpc URL ${C_ARES_URL}
  SOURCE_DIR third_party/cares/cares CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(
  get_boringssl PREFIX grpc URL ${BORINGSSL_URL}
  SOURCE_DIR third_party/boringssl CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(
  get_boringssl_with_bazel PREFIX grpc
  GIT_REPOSITORY https://boringssl.googlesource.com/boringssl
  GIT_TAG 886e7d75368e3f4fab3f4d0d3584e4abfc557755
  SOURCE_DIR third_party/boringssl-with-bazel
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_benchmark PREFIX grpc URL ${BENCHMARK_URL}
  SOURCE_DIR third_party/benchmark CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(get_gflags PREFIX grpc URL ${GFLAGS_URL}
  SOURCE_DIR third_party/gflags CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(get_googletest PREFIX grpc URL ${GOOGLETEST_URL}
  SOURCE_DIR third_party/googletest CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(get_protobuf PREFIX grpc URL ${PROTOBUF_URL}
  SOURCE_DIR third_party/protobuf CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(get_zlib PREFIX grpc URL ${ZLIB_URL}
  SOURCE_DIR third_party/zlib CONFIGURE_COMMAND "" BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Add(
  build_grpc
  PREFIX grpc
  URL ${GRPC_URL}
  PATCH_COMMAND
    ${RSYNC}
      --archive
      ${CMAKE_CURRENT_BINARY_DIR}/third_party
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build
  SOURCE_DIR grpc_build
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE 1
  BUILD_COMMAND $(MAKE) CONFIG=opt REQUIRE_CUSTOM_LIBRARIES_opt=1
  INSTALL_COMMAND ""
)

add_dependencies(build_grpc get_c_ares get_boringssl get_boringssl_with_bazel
                 get_benchmark get_gflags get_googletest get_protobuf get_zlib)

set(GRPC_INCLUDE_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc)
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
    ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc)
install(
  DIRECTORY ${GRPC_LIB_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
install(
  DIRECTORY ${GRPC_BIN_DESTINATION}/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

add_custom_command(
  TARGET build_grpc
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_INCLUDE_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_LIB_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GRPC_BIN_DESTINATION}
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/libs/opt/
      ${GRPC_LIB_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/bins/opt/
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
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/include/
      ${GRPC_INCLUDE_DESTINATION}/
  COMMAND
    ${RSYNC}
      --prune-empty-dirs --archive
      --include="*/" --include="*.h" --include="*.proto" --exclude="*"
      ${CMAKE_CURRENT_BINARY_DIR}/grpc_build/third_party/protobuf/src/
      ${GRPC_INCLUDE_DESTINATION}/
)
