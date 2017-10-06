# `grpc`: Catkinized gRPC Package

This package integrates [gRPC](https://grpc.io) into Catkin/ROS ecosystem.

## Usage

* Put this package in your workspace.

* In `CMakeLists.txt` of your package, include `gprc` in
`find_package(catkin REQUIRED COMPONENTS )` section.
Example: `find_package(catkin REQUIRED COMPONENTS grpc)`

* Incldue "`generate_proto(<proto_target> <proto_path>/<proto_name>.proto)`"
rules in your `CMakeLists.txt` (see more below).

* Use "`target_link_libraries`" to link proto target (`<proto_target>` above)
to your library or executable.

## `generate_proto` Features

### Basic Usage
`generate_proto(hello_proto proto/hello.proto)` will generate `hello.pb.h`,
`hello.pb.cc`, and `hello_pb2.py`. You can use them in your source with
`#include "your_package/hello.pb.h"` or `import your_package.hello_pb2`.
`grpc` will generate a library target called `hello_proto`,
linked with `libprotobuf` included in the `grpc` package.  You will only need to
add `target_link_libraries(my_library_or_binaray hello_proto)` to link it to
your depending targets.

### `GRPC`
The `GRPC` option also generates `.grpc.{h,cc}` and `_grpc_pb2.py` files files.
`generate_proto(hello_proto GRPC proto/hello.proto)` will also generate
`hello.grpc.pb.h`, `hello.grpc.pb.cc`, and `hello_grpc_pb2.py`.

### `SRC_BASE`
The generated C++ headers will always be in the global `include` directory in
Catkin `devel` space, and generated Python files will always be in the global
`dist-packages` directory in Catkin `devel` space. They also refer these
directory as their base. However, you can specify your source base using
`SRC_BASE`. Your source base will correspond to global `include` and
`dist-packages` directory.

The default source base is one-level above the package directory, assuming your
package directory name is identical to package name. Therefore, the generated
files will end up in a subdirectory with package name. That is the reason why
`your_package` is necessary in `#include "your_package/hello.pb.h"`. If your
source layout is `catkin_ws/src/your_package/proto/hello.proto`, by default the
source base is `catkin_ws/src`, so `your_package/proto/hello.proto` will convert
to `your_package/proto/hello{.pb.h, .pb.cc, _pb2.py}`, and end up with
`catkin_ws/devel/include/your_package/proto/hello.pb{.h, .cc}` and
`catkin_ws/devel/lib/python2.7/dist-packages/your_package/proto/hello_pb2.py`.
Please note even if your package directory name is not the same as package name,
you should use your **package name** in the include/import path.

You can use `SRC_BASE` to modify such behavior. The argument of `SRC_BASE` is
relative to **your package base directory**, `catkin_ws/src/your_package` in the
above case. If you use
`generate_proto(hello_proto GRPC SRC_BASE "." proto/hello.proto)`, you will get
`catkin_ws/devel/include/proto/hello{.pb.h, .pb.cc, .grpc.pb.h, .grpc.pb.cc}`
and `catkin_ws/devel/lib/python2.7/dist-packages/proto/hello{_grpc_pb2.py,
_pb2.py}`. Your package (and other packages in the same workspace) can use
`#include "proto/hello.pb.h` or `import proto.hello_pb2` to use them.

In another example, if you use `SRC_BASE "proto"`, you will get
`catkin_ws/devel/include/hello.pb{.h, .cc}` and
`catkin_ws/devel/lib/python2.7/dist-packages/hello_pb2.py`, and your package
(and other packages in the same workspace) can use `#include "hello.pb.h`
or `import hello_pb2`. Please note all the package will see these generated
files. Please use caution to avoid namespace pollution.


### `INCLUDE_DIRS`
`generate_proto` has a `INCLUDE_DIRS` keyword to allow searching proto in
additional directories. `INCLUDE_DIRS` accepts multiple arguments, and to
distinguish proto files to compile from include directory, a `FILES` keyword
is necessary before the file list. Example:

```
generate_proto(hello_proto INCLUDE_DIRS dir1 dir2 FILES proto/hello.proto)`
```

## Dependencies

All the dependencies are documented in `package.xml` file. These are commonly
missing dependencies: `autoconf`, `libtool`, `rsync`.

## License

This package is licensed under the 3-clause BSD License.

Copyright &copy;(2017) The Regents of the University of California, All rights
reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the University of California nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


### gRPC License
grpc (https://github.com/grpc/grpc) is licensed under Apache License, Version
2.0.

Copyright 2015 gRPC authors.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
