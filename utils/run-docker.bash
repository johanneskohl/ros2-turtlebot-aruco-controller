#!/usr/bin/bash

set -e

scriptDir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
wsDir=$(realpath "${scriptDir}"/../../..)
defaultDockerBaseName='ros2-turtlebot-aruco-controller'

videoDevice=0
rosDomainId=${ROS_DOMAIN_ID:-0}
imageName="${defaultDockerBaseName}-image"
containerName="${defaultDockerBaseName}-container"
containerUser=$(id -u)
containerGroup=$(id -g)
buildDebug=false
runBash=false
rebuild=false

function printHelp {
  echo "run-docker.bash [--help] [--video-device ID] [--ros-domain-id ID] [--image-name NAME] [--container-name NAME] [--container-user USER] [--container-group GROUP] [--build-debug] [--run-bash] [--rebuild]"
  echo "Run the workspace containing the ros2-turtlebot-aruco-controller in a docker container."
  echo
  echo "Options:"
  echo "  --help            -h          Show this message and exit."
  echo "  --video-device    -v  ID      Select the video device (/dev/video\${ID}) to channel in the image. (default: ${videoDevice})"
  echo "  --ros-domain-id   -d  ID      Set the value of ROS_DOMAIN_ID environment variable in container. (default: \$ROS_DOMAIN_ID=${rosDomainId})"
  echo "  --image-name      -i  NAME    The name of the generated docker image. (default: '${imageName}')"
  echo "  --container-name  -c  NAME    The name of the docker container. (default: '${containerName}')"
  echo "  --container-user  -u  USER    The name or id of the container user. (default: ${containerUser})"
  echo "  --container-group -g  GROUP   The name or id of the container user. (default: ${containerGroup})"
  echo "  --build-debug                 Whether to rebuild the ros2 package with visual debug output enabled"
  echo "                                Note: Ignored when --run-bash is given."
  echo "  --run-bash        -b          Run bash console instead of the node directly."
  echo "  --rebuild         -r          Force image to be rebuild, even if it already exists."
}

if ! validArgs=$(getopt \
  --options hv:d:i:c:u:g:br \
  --longoptions help,video-device:,ros-domain-id:,image-name:,container-name:,container-user:,container-group:,build-debug,run-bash,rebuild \
  -- "${@}"); then
  printHelp
  exit 1
fi

# shellcheck disable=2294
eval set -- "${validArgs[@]}"
while true; do
  case "${1}" in
    --help | -h )
      printHelp
      exit 0;;
    --video-device | -v )
      videoDevice=${2}
      shift 2;;
    --ros-domain-id | -d )
      rosDomainId=${2}
      shift 2;;
    --image-name | -i )
      imageName=${2}
      shift 2;;
    --container-name | -c )
      containerName=${2}
      shift 2;;
    --container-user | -u )
      containerUser=${2}
      shift 2;;
    --container-group | -g )
      containerGroup=${2}
      shift 2;;
    --build-debug )
      buildDebug=true
      shift 1;;
    --run-bash | -b )
      runBash=true
      shift 1;;
    --rebuild | -r )
      rebuild=true
      shift 1;;
    -- )
      shift 1
      break;;
  esac
done


if "${rebuild}" || ! docker image inspect "${imageName}" &> /dev/null; then
  echo "Missing image ${imageName}, building..."
  bash "${scriptDir}"/build-docker.bash --image-name "${imageName}"
  echo "...done."
fi

if "${buildDebug}"; then
  buildDebugToggle=ON
else
  buildDebugToggle=OFF
fi

if "${runBash}"; then
  entrypoint=(/bin/bash)
else
  parentDir=$(realpath "${wsDir}"/..)
  entrypoint=(
    /bin/bash
    /home/ubuntu/"$(realpath --no-symlinks --relative-to "${parentDir}" "${scriptDir}")"/run-node.bash
    "${videoDevice}"
    "${buildDebugToggle}"
  )
fi

containerId=$(docker container ls --no-trunc -aqf "name=${containerName}")
if [[ -n ${containerId} ]]; then
  docker container rm -f "${containerId}" > /dev/null
fi

docker run \
  -it \
  --name "${containerName}"\
  --net host \
  --device "/dev/video${videoDevice}" \
  --user "${containerUser}":"${containerGroup}" \
  --mount "type=bind,src=${wsDir},dst=/home/ubuntu/$(basename "${wsDir}")" \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix \
  --mount type=bind,src=/run/user/1000/wayland-0,dst=/run/user/1000/wayland-0 \
  --runtime nvidia \
  --gpus all \
  --env "DISPLAY=${DISPLAY}" \
  --env "ROS_DOMAIN_ID=${rosDomainId}" \
  --env "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}display,graphics" \
  --env "WAYLAND_DISPLAY=wayland-0" \
  --env "XDG_RUNTIME_DIR=/tmp/user/1000" \
  --privileged \
  "${imageName}" \
  "${entrypoint[@]}"
