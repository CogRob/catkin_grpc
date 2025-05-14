cmake_minimum_required(VERSION 3.0.2)
project(test_grpc)

# Check if the libraries installed when building the package `grpc`
# do not have name collisions with system libraries
find_library(LIB_Z z REQUIRED)
find_library(LIB_SSL ssl REQUIRED)
find_library(LIB_CRYPTO crypto REQUIRED)
find_library(LIB_PROTOC protoc)

find_package(catkin REQUIRED COMPONENTS
  grpc
  roscpp
)

function(realpath path var)
  if(CMAKE_VERSION VERSION_LESS 3.19)
    find_program(REALPATH realpath REQUIRED)
    execute_process(
      COMMAND "${REALPATH}" "${path}"
      OUTPUT_VARIABLE realpath_output
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  else()
    file(REAL_PATH "${path}" realpath_output)
  endif()
  set("${var}" "${realpath_output}" PARENT_SCOPE)
endfunction()

realpath("${grpc_DIR}/../../../lib" grpc_LIBRARY_DIR)
function(assert_grpc_library lib)
  realpath("${lib}" LIBRARY)
  string(FIND "${grpc_LIBRARY_DIR}" "${LIBRARY}" index)
  if(index LESS 0)
    message(DEBUG "${LIBRARY} is system library")
  else()
    message(FATAL_ERROR
      "${LIBRARY} is subdir of ${grpc_LIBRARY_DIR}"
      " and overrides the system library")
  endif()
endfunction()
assert_grpc_library(${LIB_Z})
assert_grpc_library(${LIB_SSL})
assert_grpc_library(${LIB_CRYPTO})
if(LIB_PROTOC)
  assert_grpc_library(${LIB_PROTOC})
endif()

catkin_package(CATKIN_DEPENDS grpc)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

generate_proto(${PROJECT_NAME}
  GRPC
  FILES protos/Header.proto grpcs/Chat.proto
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(chat_server src/chat_server.cpp)
target_link_libraries(chat_server ${catkin_LIBRARIES} ${ALL_PROTOBUF_LIBS} ${ALL_GRPC_LIBS} ${PROJECT_NAME})
add_executable(chat_client src/chat_client.cpp)
target_link_libraries(chat_client ${catkin_LIBRARIES} ${ALL_PROTOBUF_LIBS} ${ALL_GRPC_LIBS} ${PROJECT_NAME})

install(TARGETS chat_server chat_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_chat_service.test)
endif()
  
