#!/usr/bin/env bash

set -e

mkdir -p build

pushd build

# if macOS, add qt5 to path
if [[ "$OSTYPE" == "darwin"* ]]; then
    export PATH="$(brew --prefix qt5)/bin":$PATH
fi

# prepare build
cmake -DCMAKE_BUILD_TYPE=Release ..

# build plugins
cmake --build . --target ArduPilotPlugin -- -j$(nproc)
cmake --build . --target ParachutePlugin -- -j$(nproc)
cmake --build . --target PressureSensor -- -j$(nproc)
cmake --build . --target PressureSensorSystem -- -j$(nproc)

# build stream_camera
cmake --build . --target stream_camera -- -j$(nproc)

popd