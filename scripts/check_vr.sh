#!/bin/bash

# Set the STEAM environment variable
export STEAM="$HOME/.local/share/Steam"

# Set the STEAMVR environment variable
export STEAMVR="$HOME/.steam/steam/steamapps/common/SteamVR"

# Extend the LD_LIBRARY_PATH environment variable
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/libraries/openvr/lib/linux32:$HOME/libraries/openvr/lib/linux64:$HOME/.local/share/Steam/ubuntu12_32/steam-runtime/i386/lib/i386-linux-gnu:$HOME/.local/share/Steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu:$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux32:$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64:$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux32:$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux64"

${STEAMVR}/bin/linux64/vrcmd
