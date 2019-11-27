#! /bin/bash

set -x

# check for rpclib
if [ ! -f ../external/rpclib/rpclib-2.2.1/rpclib.pc.in ]; then
    >&2 echo "error, rpc.pc.in not found, please run setup.sh first and then run build.sh again"
fi

cp ../external/rpclib/rpclib-2.2.1/rpclib.pc.in AirLibWrapper/AirsimWrapper/rpclib.pc.in

if [ ! -d "linux-build" ]; then
    mkdir linux-build
fi


cd linux-build
export CC="clang-5.0"
export CXX="clang++-5.0"

# check for local cmake build created by setup.sh
if [ -d "../../cmake_build" ]; then
    if [ "$(uname)" = "Darwin" ]; then
        CMAKE="$(greadlink -f ../../cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f ../../cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

"$CMAKE" ../../cmake ../AirLibWrapper/AirsimWrapper
make -j`nproc`
if [ ! -d "../UnityDemo/Assets/Plugins" ]; then
    mkdir ../UnityDemo/Assets/Plugins
fi

if [ "$(uname)" = "Darwin" ]; then
    cp AirsimWrapper.bundle ../UnityDemo/Assets/Plugins
else
    cp libAirsimWrapper.so ../UnityDemo/Assets/Plugins
fi

cd ..
if [ -f AirLibWrapper/AirsimWrapper/rpclib.pc.in ]; then
    rm AirLibWrapper/AirsimWrapper/rpclib.pc.in
fi
