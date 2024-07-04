export openvr=~/workspace/codeopenvr
export steam=~/.steam
export steamvr=$steam/steam/steamapps/common/SteamVR

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/usr/lib/:\
/usr/lib32/:\
$openvr/lib/linux32/:\
$openvr/lib/linux64/:\
$steam/ubuntu12_32/steam-runtime/i386/lib/i386-linux-gnu/:\
$steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu/:\
$steamvr/bin/linux32/:\
$steamvr/bin/linux64/:\
$steamvr/drivers/lighthouse/bin/linux32/:\
$steamvr/drivers/lighthouse/bin/linux64/
