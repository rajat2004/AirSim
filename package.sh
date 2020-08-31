#!/bin/bash

export UE4_ROOT=~/UnrealEngine/UE_4.24
$UE4_ROOT/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
                -project="$(pwd)/Unreal/Environments/Blocks/Blocks.uproject" \
                -nop4 -nocompile -build -cook -compressed -pak -allmaps -stage \
                -archive -archivedirectory="$(pwd)/Unreal/Environments/Blocks/Packaged/Development" \
                -clientconfig=Development -noclean -utf8output
