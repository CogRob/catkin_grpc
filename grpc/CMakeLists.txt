cmake_minimum_required(VERSION 2.8.12)
include(ExternalProject)
project(grpc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

set(GRPC_TAG 1a60e6971f428323245a930031ad267bb3142ba4)
set(ABSEIL_TAG cc4bed2d74f7c8717e31f9579214ab52a9c9c610)
set(BENCHMARK_TAG 5b7683f49e1e9223cf9927b24f6fd3d6bd82e3f8)
set(BLOATY_TAG 73594cde8c9a52a102c4341c244c833aa61b9c06)
set(BORINGSSL_TAG b29b21a81b32ec273f118f589f46d56ad3332420)
set(BORINGSSL_WITH_BAZEL_TAG 8149b351bf797bd80e063787886b7618f508e451)
set(C_ARES_TAG 3be1924221e1326df520f8498d704a5c4c8d0cce)
set(GFLAGS_TAG 30dbc81fb5ffdc98ea9b14b1918bfe4e8779b26e)
set(GOOGLETEST_TAG ec44c6c1675c25b9827aacd08c02433cccde7780)
set(LIBCXX_TAG 6599cac0965be8e5a835ab7a5684bbef033d5ad0)
set(LIBCXXABI_TAG 9245d481eb3e890f708ff2d7dadf2a10c04748ba)
set(PROTOBUF_TAG 48cb18e5c419ddd23d9badcfe4e9df7bde1979b2)
set(ZLIB_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f)

set(GRPC_URL https://github.com/grpc/grpc/archive/${GRPC_TAG}.tar.gz)
set(ABSEIL_URL https://github.com/abseil/abseil-cpp/archive/${ABSEIL_TAG}.tar.gz)
set(BENCHMARK_URL https://github.com/google/benchmark/archive/${BENCHMARK_TAG}.tar.gz)
set(BLOATY_URL https://github.com/google/bloaty/archive/${BLOATY_TAG}.tar.gz)
set(BORINGSSL_URL https://github.com/google/boringssl/archive/${BORINGSSL_TAG}.tar.gz)
set(BORINGSSL_WITH_BAZEL_URL https://github.com/google/boringssl/archive/${BORINGSSL_WITH_BAZEL_TAG}.tar.gz)
set(C_ARES_URL https://github.com/c-ares/c-ares/archive/${C_ARES_TAG}.tar.gz)
set(GFLAGS_URL https://github.com/gflags/gflags/archive/${GFLAGS_TAG}.tar.gz)
set(GOOGLETEST_URL https://github.com/google/googletest/archive/${GOOGLETEST_TAG}.tar.gz)
set(LIBCXX_URL https://github.com/llvm-mirror/libcxx/archive/${LIBCXX_TAG}.tar.gz)
set(LIBCXXABI_URL https://github.com/llvm-mirror/libcxxabi/archive/${LIBCXXABI_TAG}.tar.gz)
set(PROTOBUF_URL https://github.com/protocolbuffers/protobuf/archive/${PROTOBUF_TAG}.tar.gz)
set(ZLIB_URL https://github.com/madler/zlib/archive/${ZLIB_TAG}.tar.gz)

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
  get_boringssl PREFIX grpc URL ${BORINGSSL_URL}
  SOURCE_DIR third_party/boringssl
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_boringssl_with_bazel PREFIX grpc URL ${BORINGSSL_WITH_BAZEL_URL}
  SOURCE_DIR third_party/boringssl-with-bazel
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_benchmark PREFIX grpc URL ${BENCHMARK_URL}
  SOURCE_DIR third_party/benchmark
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_libcxx PREFIX grpc URL ${LIBCXX_URL}
  SOURCE_DIR third_party/libcxx
  CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND "")

ExternalProject_Add(
  get_libcxxabi PREFIX grpc URL ${LIBCXXABI_URL}
  SOURCE_DIR third_party/libcxxabi
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
  get_gflags get_googletest get_protobuf get_zlib get_abseil get_bloaty
  get_libcxx get_libcxxabi)

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
