#!/bin/bash

SCRIPTPATH=$(dirname "$(realpath $0)")
LIB_DIR=$(realpath "$SCRIPTPATH/../../..")

export OPENVR="$LIB_DIR/libraries/openvr"
echo "OPENVR path: $OPENVR"

# export OPENVR=~/armstrong/interbotix_ws/libraries
export STEAM=~/.steam
export STEAMVR=$STEAM/steam/steamapps/common/SteamVR
echo "STEAMVR path: $STEAMVR"

# Set the LD_LIBRARY_PATH to include the necessary shared libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/usr/lib/:\
/usr/lib32/:\
$OPENVR/lib/linux32/:\
$OPENVR/lib/linux64/:\
$STEAM/ubuntu12_32/steam-runtime/i386/lib/i386-linux-gnu/:\
$STEAM/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu/:\
$STEAMVR/bin/linux32/:\
$STEAMVR/bin/linux64/:\
$STEAMVR/drivers/lighthouse/bin/linux32/:\
$STEAMVR/drivers/lighthouse/bin/linux64/
