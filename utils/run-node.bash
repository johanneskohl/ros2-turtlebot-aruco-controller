#!/usr/bin/bash

videoDeviceId=${1:-4}

set -em
scriptDir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
wsDir=$(realpath "${scriptDir}"/../../..)
nodeName="/turtlebot_aruco_controller"

function aruco-controller-node-online {
  ros2 node list | grep -qF "${nodeName}"
}

# since the shutdown command can sometimes hang, here is a workaround
function shutdown-aruco-controller {
  ros2 lifecycle set "${nodeName}" shutdown &
  while aruco-controller-node-online; do
    sleep 1
  done
}


############ Stupid fix for a stupid bug, see readme for more info ############
if ! v4l2-ctl --list-devices &> /dev/null; then
  sudo chmod 777 /dev/video"${videoDeviceId}"
fi
###############################################################################

if [[ -z ${ROS_VERSION} ]]; then
  # shellcheck disable=1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
source "${wsDir}/install/setup.bash"

if aruco-controller-node-online; then
  shutdown-aruco-controller
fi
ros2 run turtlebot_aruco_controller main \
  --ros-args \
    --log-level DEBUG \
    --param camera.id:="${videoDeviceId}" \
    --param linear_scaling_factor:=-2.0 \
    -- &

while ! aruco-controller-node-online; do
  sleep 1
done

if [ "$(ros2 lifecycle set "${nodeName}" configure)" != "Transitioning successful" ]; then
  shutdown-aruco-controller
fi
if [ "$(ros2 lifecycle set "${nodeName}" activate)" != "Transitioning successful" ]; then
  shutdown-aruco-controller
fi

jobsPattern='^\[([0-9]+)\].? *[^ ]* *ros2 run turtlebot_aruco_controller main --ros-args --log-level DEBUG --param camera.id:=4 -- &'
if [[ $(jobs) =~ ${jobsPattern} ]]; then
  jobId=${BASH_REMATCH[1]}
  # TODO: no jobservice
  fg "%${jobId}"
fi
