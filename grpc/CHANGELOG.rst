^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grpc
^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2017-11-20)
------------------
* Depend on zlib (`#23 <https://github.com/CogRob/catkin_grpc/issues/23>`_) and link protobuf with zlib (`#22 <https://github.com/CogRob/catkin_grpc/issues/22>`_) (`#24 <https://github.com/CogRob/catkin_grpc/issues/24>`_)
* Depend on absolute path of .proto file (`#21 <https://github.com/CogRob/catkin_grpc/issues/21>`_)
  This should fix `#20 <https://github.com/CogRob/catkin_grpc/issues/20>`_. The original concern was for some reason cmake re-generate protos if .proto is not within src. Initial test shows this is not the case, but should do more tests.
* Contributors: Shengye Wang

0.0.5 (2017-10-06)
------------------
* Fixes a problem in default SRC_BASE (`#18 <https://github.com/CogRob/catkin_grpc/issues/18>`_)
  * Fix an issue when package directory is not the same as package name, protoc fails to generate files in the correct path
  * Improve the copy src/proto command
  * Revert "Improve the copy src/proto command"
  This reverts commit adfca498b7b3b00fb4c350675d3b4dbcb154fe75.
  * Fixes a problem that src in include is not installed
* Contributors: Shengye Wang

0.0.4 (2017-10-02)
------------------
* Adds INCLUDE_DIRS and expose grpc/src/proto (`#14 <https://github.com/CogRob/catkin_grpc/issues/14>`_)
  * Allows generate_proto to have specify include_dirs
  * Allow grpc's build-in protos to be used by clients
* Minor documentation fix. (`#13 <https://github.com/CogRob/catkin_grpc/issues/13>`_)
* Contributors: Shengye Wang

0.0.3 (2017-09-30)
------------------
* Allows CMake to choose dynamic/static linking on grpc libraries
* Some fix to grpc package. (`#11 <https://github.com/CogRob/catkin_grpc/issues/11>`_)
  * Use all embedded third_party software, use grpc 1.6.x, use all static linking to avoid conflicts, move include dir
  * Use HTTPS download instead of GIT to speedup downloading
  * Missing libs
  * Adds boringssl with Bazel in third_party
* Contributors: Shengye Wang

0.0.2 (2017-09-05)
------------------
* Lower version requirement for cmake so that the package can be built under
  trusty.
* Contributors: Shengye Wang

0.0.1 (2017-09-04)
------------------
* Pre-release commit. (`#1 <https://github.com/CogRob/catkin_grpc/issues/1>`_)
* Contributors: Shengye Wang
