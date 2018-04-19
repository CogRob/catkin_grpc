cmake_minimum_required(VERSION 2.8.12)
include(ExternalProject)
project(grpc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

set(GRPC_TAG 66b9770a8ad326c1ee0dbedc5a8f32a52a604567)
set(GRPC_URL https://github.com/grpc/grpc/archive/${GRPC_TAG}.tar.gz)

set(C_ARES_TAG 3be1924221e1326df520f8498d704a5c4c8d0cce)
set(C_ARES_URL https://github.com/c-ares/c-ares/archive/${C_ARES_TAG}.tar.gz)

set(BORINGSSL_TAG a20bb7ff8bb5057065a2e7941249773f9676cf45)
set(BORINGSSL_URL https://github.com/google/boringssl/archive/${BORINGSSL_TAG}.tar.gz)

set(BENCHMARK_TAG 5b7683f49e1e9223cf9927b24f6fd3d6bd82e3f8)
set(BENCHMARK_URL https://github.com/google/benchmark/archive/${BENCHMARK_TAG}.tar.gz)

set(GFLAGS_TAG 30dbc81fb5ffdc98ea9b14b1918bfe4e8779b26e)
set(GFLAGS_URL https://github.com/gflags/gflags/archive/${GFLAGS_TAG}.tar.gz)

set(GOOGLETEST_TAG ec44c6c1675c25b9827aacd08c02433cccde7780)
set(GOOGLETEST_URL https://github.com/google/googletest/archive/${GOOGLETEST_TAG}.tar.gz)

set(PROTOBUF_TAG 2761122b810fe8861004ae785cc3ab39f384d342)
set(PROTOBUF_URL https://github.com/google/protobuf/archive/${PROTOBUF_TAG}.tar.gz)

set(ZLIB_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f)
set(ZLIB_URL https://github.com/madler/zlib/archive/${ZLIB_TAG}.tar.gz)

set(ABSEIL_TAG cc4bed2d74f7c8717e31f9579214ab52a9c9c610)
set(ABSEIL_URL https://github.com/abseil/abseil-cpp/archive/${ABSEIL_TAG}.tar.gz)

set(BLOATY_TAG 73594cde8c9a52a102c4341c244c833aa61b9c06)
set(BLOATY_URL https://github.com/google/bloaty/archive/${BLOATY_TAG}.tar.gz)

catkin_package(
  CFG_EXTRAS generate_proto.cmake
)

find_program(RSYNC rsync)
if(NOT RSYNC)
  message(SEND_ERROR "Cannot find rsync.")
endif(NOT RSYNC)

ExternalProject_Add(get_c_ares PREFIX grpc URL ${C_ARES_URL}
  SOURCE_DIR third_party/cares/cares
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_boringssl_with_bazel PREFIX grpc
  GIT_REPOSITORY https://github.com/google/boringssl.git
  GIT_TAG 886e7d75368e3f4fab3f4d0d3584e4abfc557755
  SOURCE_DIR third_party/boringssl-with-bazel
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_boringssl PREFIX grpc URL ${BORINGSSL_URL}
  SOURCE_DIR third_party/boringssl
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_benchmark PREFIX grpc URL ${BENCHMARK_URL}
  SOURCE_DIR third_party/benchmark
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_gflags PREFIX grpc URL ${GFLAGS_URL}
  SOURCE_DIR third_party/gflags
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_googletest PREFIX grpc URL ${GOOGLETEST_URL}
  SOURCE_DIR third_party/googletest
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_protobuf PREFIX grpc URL ${PROTOBUF_URL}
  SOURCE_DIR third_party/protobuf
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_bloaty PREFIX grpc URL ${BLOATY_URL}
  SOURCE_DIR third_party/bloaty
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_abseil PREFIX grpc URL ${ABSEIL_URL}
  SOURCE_DIR third_party/abseil-cpp
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(get_zlib PREFIX grpc URL ${ZLIB_URL}
  SOURCE_DIR third_party/zlib
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

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

add_dependencies(
  build_grpc get_c_ares get_boringssl get_boringssl_with_bazel get_benchmark
  get_gflags get_googletest get_protobuf get_zlib get_abseil get_bloaty)

set(GRPC_INCLUDE_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/grpc)
set(GRPC_LIB_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set(GRPC_BIN_DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

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
