#!/usr/bin/bash

videoDeviceId=${1:-4}
buildDebug=${2:-OFF}

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


if ! v4l2-ctl --list-devices &> /dev/null; then
  sudo setfacl --set=user::rw-,user:1000:rw-,group::rw-,mask::rw-,other::--- "/dev/video${videoDeviceIdx}"
fi

if [[ -z ${ROS_VERSION} ]]; then
  # shellcheck disable=1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
colcon build \
  --symlink-install \
  --packages-select \
    turtlebot_aruco_controller \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DTAC_DEBUG=${buildDebug}
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
