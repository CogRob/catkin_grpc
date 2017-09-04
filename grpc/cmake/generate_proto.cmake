if(CMAKE_VERSION VERSION_LESS "3.1")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
    message(SEND_ERROR "Error: Can not enable C++ 11 for gRPC")
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_program(FIND find)
if(NOT FIND)
  message(SEND_ERROR "Cannot find find.")
endif(NOT FIND)

set(GRPC_BIN_DIR ${grpc_PREFIX}/${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/grpc)

unset(PROTOBUF_PROTOC_EXECUTABLE CACHE)
find_program(PROTOBUF_PROTOC_EXECUTABLE protoc
             PATHS ${GRPC_BIN_DIR}/protobuf NO_DEFAULT_PATH)
message(STATUS "Found protoc at: ${PROTOBUF_PROTOC_EXECUTABLE}")

unset(GRPC_CPP_PLUGIN CACHE)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin
             PATHS ${GRPC_BIN_DIR} NO_DEFAULT_PATH)
message(STATUS "Found grpc_cpp_plugin at: ${GRPC_CPP_PLUGIN}")

unset(GRPC_PYTHON_PLUGIN CACHE)
find_program(GRPC_PYTHON_PLUGIN grpc_python_plugin
             PATHS ${GRPC_BIN_DIR} NO_DEFAULT_PATH)
message(STATUS "Found grpc_python_plugin at: ${GRPC_PYTHON_PLUGIN}")

set(GRPC_LIB_DIR ${grpc_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
find_library(
    LIBPROTOBUF protobuf PATHS ${GRPC_LIB_DIR}/protobuf NO_DEFAULT_PATH)
message(STATUS "Found libprotobuf: ${LIBPROTOBUF}")

set(PROTOBUF_INCLUDE_DIR ${grpc_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

set(GENERATE_PROTO_STAMP_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(GENERATE_PROTO_CC_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(GENERATE_PROTO_CC_HDR_OUTPUT_DIR
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
set(GENERATE_PROTO_PY_OUTPUT_DIR
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

function(generate_proto PROTO_TARGET_NAME)
  cmake_parse_arguments(protogen "GRPC" "SRC_BASE" "" ${ARGN})
  set(WITH_GRPC ${protogen_GRPC})
  set(SRC_BASE ${protogen_SRC_BASE})
  set(PROTO_FILES "${protogen_UNPARSED_ARGUMENTS}")

  if(NOT PROTO_FILES)
    message(SEND_ERROR "Error: generate_proto() called without any proto files")
    return()
  endif()

  if(NOT SRC_BASE)
    get_filename_component(
      SRC_RELATIVE_BASE_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)
  else()
    set(SRC_RELATIVE_BASE_DIR "${PROJECT_SOURCE_DIR}/${SRC_BASE}")
  endif()

  file(MAKE_DIRECTORY ${GENERATE_PROTO_CC_OUTPUT_DIR})
  file(MAKE_DIRECTORY ${GENERATE_PROTO_CC_HDR_OUTPUT_DIR})
  file(MAKE_DIRECTORY ${GENERATE_PROTO_PY_OUTPUT_DIR})

  unset(ALL_STAMP_TARGETS)
  unset(PROTOGEN_GENERATED_LIST)
  unset(PROTOGEN_GENERATED_LIST)
  unset(PROTOGEN_CC_GENERATED_LIST)

  foreach(FIL ${PROTO_FILES})
    get_filename_component(ABS_FILE_PATH ${FIL} ABSOLUTE)
    get_filename_component(FILE_BASENAME ${FIL} NAME_WE)
    # FILE_RELPATH_BASE is the relative file path to base.
    file(RELATIVE_PATH FILE_RELPATH_BASE
         ${SRC_RELATIVE_BASE_DIR} ${ABS_FILE_PATH})
    file(RELATIVE_PATH FILE_RELPATH_PROJECT_SRC
         ${PROJECT_SOURCE_DIR} ${ABS_FILE_PATH})
    get_filename_component(DIR_FILE ${FILE_RELPATH_BASE} DIRECTORY)

    # DEST_STAMP_FILE is stamp file to mark execution of protoc.
    set(
      DEST_STAMP_FILE
      "${GENERATE_PROTO_STAMP_OUTPUT_DIR}/${DIR_FILE}/${FILE_BASENAME}.pbstamp")
    # DEST_CC_FILE_WE is .h/.cc output file path in build space w/o extension.
    set(DEST_CC_FILE_WE
        "${GENERATE_PROTO_CC_OUTPUT_DIR}/${DIR_FILE}/${FILE_BASENAME}")
    # DEST_CC_FILE_WE is .h output file path in devel space w/o extension.
    set(DEST_CC_HDR_FILE_WE
        "${GENERATE_PROTO_CC_HDR_OUTPUT_DIR}/${DIR_FILE}/${FILE_BASENAME}")
    # DEST_PY_FILE_WE is .py output file path w/o _pb2.py or grpc_pb2.py suffix.
    set(DEST_PY_FILE_WE
        "${GENERATE_PROTO_PY_OUTPUT_DIR}/${DIR_FILE}/${FILE_BASENAME}")

    unset(PROTOC_OUTPUT_FILES)
    unset(PROTOC_EXTRA_ARGS)

    set(CURRENT_GENERATED_CC_LIST
        "${DEST_CC_FILE_WE}.pb.cc" "${DEST_CC_FILE_WE}.pb.h")
    set(CURRENT_GENERATED_PY_LIST "${DEST_PY_FILE_WE}_pb2.py")
    set(CURRENT_GENERATED_OTHER_LIST "${DEST_CC_HDR_FILE_WE}.pb.h")

    if(WITH_GRPC)
      list(APPEND CURRENT_GENERATED_CC_LIST
           "${DEST_CC_FILE_WE}.grpc.pb.cc" "${DEST_CC_FILE_WE}.grpc.pb.h")
      list(APPEND CURRENT_GENERATED_PY_LIST "${DEST_PY_FILE_WE}_grpc_pb2.py")
      set(CURRENT_GENERATED_OTHER_LIST "${DEST_CC_HDR_FILE_WE}.grpc.pb.h")
      list(APPEND PROTOC_EXTRA_ARGS
           "--grpc_out=${GENERATE_PROTO_CC_OUTPUT_DIR}"
           "--python-grpc_out=${GENERATE_PROTO_PY_OUTPUT_DIR}"
           "--plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}"
           "--plugin=protoc-gen-python-grpc=${GRPC_PYTHON_PLUGIN}")
    endif()

    set(PROTOC_OUTPUT_FILES ${DEST_STAMP_FILE} ${CURRENT_GENERATED_OTHER_LIST}
        ${CURRENT_GENERATED_CC_LIST} ${CURRENT_GENERATED_PY_LIST})

    list(APPEND PROTOGEN_CC_GENERATED_LIST ${CURRENT_GENERATED_CC_LIST})
    list(APPEND PROTOGEN_GENERATED_LIST ${DEST_STAMP_FILE}
         ${CURRENT_GENERATED_CC_LIST} ${CURRENT_GENERATED_PY_LIST}
         ${CURRENT_GENERATED_OTHER_LIST})

    add_custom_command(
      OUTPUT ${DEST_STAMP_FILE}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${GENERATE_PROTO_CC_OUTPUT_DIR}
             --python_out=${GENERATE_PROTO_PY_OUTPUT_DIR}
             -I${SRC_RELATIVE_BASE_DIR}
             -I${PROTOBUF_INCLUDE_DIR}
             ${PROTOC_EXTRA_ARGS}
             ${ABS_FILE_PATH}
      COMMAND ${CMAKE_COMMAND}
        ARGS -E touch ${DEST_STAMP_FILE}
      DEPENDS ${FILE_RELPATH_PROJECT_SRC}
      COMMENT "Running protocol buffer compiler on \"${FIL}\"."
      VERBATIM
    )

    string(REGEX REPLACE "[^0-9a-zA-Z]" "_"
           DEST_STAMP_TARGET "${DIR_FILE}/${FILE_BASENAME}_pbstamp")
    list(APPEND ALL_STAMP_TARGETS ${DEST_STAMP_TARGET})
    add_custom_target(${DEST_STAMP_TARGET} DEPENDS ${DEST_STAMP_FILE})

    # Copies .h file to devel space.
    add_custom_command(
      TARGET ${DEST_STAMP_TARGET}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND}
        ARGS -E copy "${DEST_CC_FILE_WE}.pb.h" "${DEST_CC_HDR_FILE_WE}.pb.h"
      DEPENDS ${DEST_STAMP_FILE}
    )
    if(WITH_GRPC)
      add_custom_command(
        TARGET ${DEST_STAMP_TARGET}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
          ARGS -E copy
            "${DEST_CC_FILE_WE}.grpc.pb.h" "${DEST_CC_HDR_FILE_WE}.grpc.pb.h"
        DEPENDS ${DEST_STAMP_FILE}
      )
    endif()
  endforeach()

  set_source_files_properties(
    ${PROTOGEN_GENERATED_LIST} PROPERTIES GENERATED TRUE)

  add_custom_target(
    "${PROTO_TARGET_NAME}_py_init" ALL
    COMMAND ${FIND} . -type d -execdir touch {}/__init__.py ";"
    WORKING_DIRECTORY ${GENERATE_PROTO_PY_OUTPUT_DIR}
    DEPENDS ${ALL_STAMP_TARGETS}
    VERBATIM
  )

  include_directories(BEFORE ${PROTOBUF_INCLUDE_DIR})
  include_directories(${GENERATE_PROTO_CC_OUTPUT_DIR})
  add_library(${PROTO_TARGET_NAME} ${PROTOGEN_CC_GENERATED_LIST})
  add_dependencies(${PROTO_TARGET_NAME} ${ALL_STAMP_TARGETS})
  target_link_libraries(${PROTO_TARGET_NAME} ${LIBPROTOBUF})

endfunction()