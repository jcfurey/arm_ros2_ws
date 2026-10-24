# ROS 2 arm workspace

ROS 2 **Lyrical**, **Gazebo Jetty**, and **MoveIt 2** on an Apple Silicon Mac.
The default environment is Ubuntu 26.04 in a native `linux/arm64` Docker
container. RViz and Gazebo appear in a browser desktop, so no XQuartz setup is
needed. A simple six-axis arm is included as a starting point; it does not
represent a particular physical robot.

## Quick start on this Mac

Install/start Docker Desktop, then run from this repository:

```sh
make image
make up
```

Open [the desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale),
then click **Connect** if prompted. The container builds the three local packages,
starts Gazebo, spawns the arm, activates its controllers, and starts MoveIt and
RViz. The first image build downloads several GB of dependencies. Give Docker
enough memory for Gazebo and RViz (8 GB is a useful starting point).

In RViz's **MotionPlanning** panel, select the `arm` planning group. In the
**Planning** tab, choose `ready` as the goal state, click **Plan**, then
**Execute**. The arm should move in both Gazebo and RViz. You can also move the
interactive goal marker. The named `home` state returns the arm upright.

```sh
make logs       # startup progress and ROS logs; Ctrl-C leaves the service running
make shell      # a ROS-sourced shell in the running container
make down       # stop the demo; keep build artifacts
```

The browser desktop is published only on this Mac's loopback interface. It has
no login. Rendering uses Mesa software rendering because Docker Desktop does
not pass the Apple GPU through to Linux containers; expect lower frame rates
than native Linux. A Linux ARM64 VM with accelerated graphics is another option
if rendering performance becomes limiting.

## Other launch modes

```sh
make mock       # RViz + MoveIt + ros2_control mock hardware, without Gazebo
make headless   # Gazebo physics + MoveIt, without either GUI
make up         # switch back to both GUIs and Gazebo
```

These commands update the same Compose service, replacing the previous mode.
For example, if the desktop port is occupied:

```sh
DESKTOP_PORT=6081 make up
```

On an Intel Linux machine, use `DOCKER_PLATFORM=linux/amd64 make image up`.
The Docker base tag is distribution-specific but follows upstream updates;
set `ROS_IMAGE` to an image digest if you need an immutable base.

## Verify planning and execution

```sh
make smoke-mock
make smoke
```

Each command starts a separate, temporary headless stack with its own ROS
domain. It checks that both controllers are active and all six joint states
arrive, asks MoveIt to plan **and execute** `ready` and `home`, and checks the
measured joint positions. The Gazebo check also requires an advancing `/clock`.
The test stops its stack when complete. It shares the build volumes with the
main service, so run builds and smoke checks sequentially.

To check an already-running demo (this moves the simulated arm):

```sh
make shell
ros2 run arm_bringup check_stack --mode gazebo  # use --mode mock for mock mode
```

## Development

```text
src/
  arm_description/     # URDF/xacro, inertias, limits, mock/Gazebo interfaces
  arm_moveit_config/   # SRDF, KDL IK, OMPL, controllers, RViz configuration
  arm_bringup/         # launch, local Gazebo world, live integration check
docker/               # ROS environment and browser desktop startup
scripts/              # build, demo, and smoke-test entry points
```

Edit files on macOS. The repository is mounted at `/ws`; `build`, `install`, and
`log` use Docker named volumes, keeping Linux artifacts separate from native
macOS artifacts. The application runs as the container user `ros`.

After changes, stop the demo, rebuild, then start it again:

```sh
make down
make build
make up
```

When adding package dependencies, declare them in `package.xml`. Add binary
dependencies to the Dockerfile and run `make image` to persist them. For a
temporary development install, use a container shell:

```sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
```

To use your own robot, replace the xacro and update the SRDF planning group,
collision exclusions, joint names, joint limits, controller lists, and IK
configuration together. `moveit_setup_assistant` is installed in the image.
The demo uses `world` as its fixed frame, `tool0` as its tip, and position
commands through `/arm_controller/follow_joint_trajectory`.

The launch file starts MoveIt after controller activation; it starts the
controller spawner after Gazebo creates the robot. Gazebo's plugin owns the
controller manager in Gazebo mode. Mock mode starts `ros2_control_node` instead.

## Native ROS installation

The existing installation at `~/ros2_lyrical/install/setup.zsh` can remain in
use for native macOS development. It currently provides RViz and xacro but
lacks the MoveIt/Gazebo/control packages required by this demo. The container
provides those dependencies without changing the native installation.

On a Linux host or VM with a complete Lyrical installation:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
bash scripts/build.sh
source install/setup.bash
ros2 launch arm_bringup demo.launch.py mode:=gazebo gui:=true rviz:=true
# Or: mode:=mock rviz:=true
```

Native host ROS nodes are not automatically connected to ROS nodes inside
Docker Desktop. Use `make shell` for CLI tools; all demo nodes use Cyclone DDS,
domain `42`, and localhost discovery inside the same container.

## Troubleshooting

- **Docker connection error:** start Docker Desktop and wait for its engine.
- **Blank desktop:** inspect `make logs` and `/tmp/desktop.log` in the container.
  Confirm you opened `/vnc.html`, then connect.
- **Slow graphics:** use `make mock` or `make headless`; software rendering is
  expected on Docker Desktop for Mac.
- **No simulated motion:** Gazebo must be playing; the launch starts it with
  `-r`. Inspect `ros2 control list_controllers` in `make shell`.
- **Clean rebuild:** `make down`, then `docker compose down -v` removes the
  generated Docker build/install/log volumes. Your source files remain.

## Upstream references

- [ROS Docker images and supported architectures](https://hub.docker.com/_/ros)
- [Gazebo/ROS version pairings](https://gazebosim.org/docs/latest/ros_installation/)
- [MoveIt 2 binary installation](https://moveit.ai/install-moveit2/binary/)
- [Lyrical gz_ros2_control integration](https://control.ros.org/lyrical/doc/gz_ros2_control/doc/index.html)
