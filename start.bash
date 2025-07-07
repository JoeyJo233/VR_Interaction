#!/bin/bash
#
# Under Linux, starts two instances (nodes) of the default solution in
# lab2 of TNM116, using the configuration that sets up VRPN tracking,
# and starts the VRPN tracking simulator.
#

SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_PATH

CONFIG=config/desktop-2-node-time-based-tracking.xml
#CONFIG=config/desktop-2-node-VRPN-tracking.xml

./build_lnx/main --config $CONFIG --param SyncNode.localPeerIdx=0 &
./build_lnx/main --config $CONFIG --param SyncNode.localPeerIdx=1 &

#H3DLoad urn:candy:x3d/VRPNServer.x3d &
