^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grpc
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
