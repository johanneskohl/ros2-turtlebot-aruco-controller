# ROS2 Turtlebot3 Aruco Controller

A controller node for a Turtlebot3 robot, or any robot listening to `geometry_msgs/TwistStamped` messages, that emits commands based on the positioning of an aruco marker in front of a camera.

## Installing

Requirements:
- ROS2 Jazzy Jalico
- OpenCV (tested for 4.6.0) with V4L Backend (should be installed with OpenCV automatically)

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/johanneskohl/ros2-turtlebot-aruco-controller.git
```

### Running in Container

```bash
bash src/ros2-turtlebot-aruco-controller/utils/run-docker.bash
```

This builds a docker image containing all necessities, runs it, builds the workspace inside and then starts the node.
For information on its command line arguments, use the `--help` argument or see the documentation below.

### Running locally

```bash
source /opt/ros/jazzy/setup.bash
bash src/ros2-turtlebot-aruco-controller/utils/run-node.bash
```

This expects all requirements to be installed and available locally.
For more information about the useage of this script see below.

## Usage

### Scripts

All scripts described here can be found at [`ros2-turtlebot-aruco-controller/utils/`](./utils/).

#### [run-docker.bash](./utils/run-docker.bash)

Runs the docker image with the aruco controller node automatically.
If the host device has multiple video devices, specify the required one with `--video-device`.
If `--run-bash` is specified, the container runs an interactive bash session in which the node may be manually started.
Otherwise the aruco controller node gets started using [`run-node.bash`](#run-nodebash).

Signature:
`run-docker.bash [--help] [--video-device ID] [--ros-domain-id ID] [--image-name NAME] [--container-name NAME] [--container-user USER] [--container-group GROUP] [--run-bash] [--rebuild]`

Options:
 Long Option        |  Short Option  |  Argument  |  Description
 ------------------ | -------------- | ---------- | --------------------------------------------------------------------------------------------------------------------
 --help             |  -h            |            |  Show the help message and exit.
 --video-device     |  -v            |  ID        |  Select the video device (/dev/video${ID}) to channel in the image. (default: 0)
 --ros-domain-id    |  -d            |  ID        |  Set the value of ROS_DOMAIN_ID environment variable in container. (default: ${ROS_DOMAIN_ID:-0})
 --image-name       |  -i            |  NAME      |  The name of the generated docker image. (default: 'ros2-turtlebot-aruco-controller-image')
 --container-name   |  -c            |  NAME      |  The name of the docker container. (default: 'ros2-turtlebot-aruco-controller-container')
 --container-user   |  -u            |  USER      |  The name or id of the container user. (default: id of executing user)
 --container-group  |  -g            |  GROUP     |  The name or id of the container user. (default: id of executing users group)
 --build-debug      |                |            |  Whether to rebuild the ros2 package with visual debug output enabled. <br/> Note: Ignored when --run-bash is given.
 --run-bash         |  -b            |            |  Run bash console instead of the node directly.
 --rebuild          |  -r            |            |  Force image to be rebuild, even if it already exists.

#### [run-node.bash](./utils/run-node.bash)

This is mainly a helper script invoked by [`run-docker.bash`](#run-dockerbash).
It starts the node, pushes the process to the background to automatically perform state changes until the node is active, then returns the node back to the foreground.
It may be manually invoked when running the docker image with the `--run-bash` option.
Note, that to specify the `BUILD_DEBUG` argument, the `VIDEO_DEVICE_ID` argument needs to be defined.

Signature:
`run-node.bash [VIDEO_DEVICE_ID] [BUILD_DEBUG]`

Options:
 Long Option  |  Short Option  |  Argument         |  Description
 ------------ | -------------- | ----------------- | ---------------------------------------------------------------------------------------------
 <i></i>      |                |  VIDEO_DEVICE_ID  |  Select the video device (/dev/video${VIDEO_DEVICE_ID}) to channel in the image. (default: 0)
 <i></i>      |                |  BUILD_DEBUG      |  Whether to rebuild the ros2 package with visual debug output enabled.

#### [build-docker.bash](./utils/build-docker.bash)

Builds the docker image used to run the node.
This script is usually invoked by [`run-docker.bash`](#run-dockerbash) on demand or when `--rebuild` is specified.

Signature:
`build-docker.bash [--help] [--image-name NAME]`

Options:
 Long Option   |  Short Option  |  Argument  |  Description
 ------------- | -------------- | ---------- | -------------------------------------------------------------------------------------------
 --help        |  -h            |            |  Show the help message and exit.
 --image-name  |  -i  NAME      |            |  The name of the generated docker image. (default: 'ros2-turtlebot-aruco-controller-image')
