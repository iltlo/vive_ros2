#!/usr/bin/env bash

# Set up environment variables
export STEAM="$HOME/.local/share/Steam"
export STEAMVR="$HOME/.steam/steam/steamapps/common/SteamVR"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$STEAM/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu:$STEAMVR/bin/linux64:$STEAMVR/drivers/lighthouse/bin/linux64"


export openvr=~/workspace/codeopenvr
export steam=~/.steam
export steamvr=$steam/steam/steamapps/common/SteamVR

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\

$steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu/:\
$steamvr/bin/linux64/:\
$steamvr/drivers/lighthouse/bin/linux64/


# Debug: Print the environment variable
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"


# Launch the VR server
${STEAMVR}/bin/linux64/vrserver --keepalive