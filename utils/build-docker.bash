#!/usr/bin/bash

set -e

scriptDir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
wsDir=$(realpath "${scriptDir}"/../../..)
defaultDockerBaseName='ros2-turtlebot-aruco-controller'

imageName="${defaultDockerBaseName}-image"

function printHelp {
  echo "build-docker.bash [--help] [--image-name NAME]"
  echo "Build the docker image to run ros2-turtlebot-aruco-controller containerised."
  echo
  echo "Options:"
  echo "  --help            -h          Show this message and exit."
  echo "  --image-name      -i  NAME    The name of the generated docker image. (default: '${imageName}')"
}

if ! validArgs=$(getopt \
  --options hi: \
  --longoptions help,image-name: \
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
    --image-name | -i )
      imageName=${2}
      shift 2;;
    -- )
      shift 1
      break;;
  esac
done


docker buildx build \
  --load \
  --build-arg BUILDKIT_INLINE_CACHE=1 \
  --file "${scriptDir}/Dockerfile" \
  --tag "${imageName}" \
  "${wsDir}"
