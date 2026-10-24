# ROS 2 arm workspace

ROS 2 **Lyrical**, **Gazebo Jetty**, and **MoveIt 2** on an Apple Silicon Mac.
The default environment is Ubuntu 26.04 in a native `linux/arm64` Docker
container. Gazebo runs **headless**. The desktop **Lichtblick** application
connects through Foxglove Bridge at **`ws://localhost:8765`**.
The default robot is a **UFACTORY xArm6**, using the manufacturer's
meshes, kinematics, inertias, and joint limits. The original primitive arm is
also available.

## Quick start on this Mac

Clone the workspace with its registered dependencies:

```sh
git clone --recurse-submodules https://github.com/jcfurey/arm_ros2_ws.git
cd arm_ros2_ws
```

Install/start Docker Desktop, then run from this repository:

```sh
make image
make up
```

On first startup the container initializes any missing source submodules,
builds the workspace, and starts Gazebo, the controllers, MoveIt, two Ousters,
and Foxglove Bridge. The first image and C++ source build take several minutes.
Compilation defaults to one job for this 16 GB Mac; runtime CPU raycasting
uses two OpenMP threads per process.

In Lichtblick, choose **File → Open connection → Foxglove WebSocket**, enter
`ws://localhost:8765`, and open the connection. In **Layouts → Import from
file…**, import [`lichtblick/workcell.json`](lichtblick/workcell.json). It
includes the scene, both clouds, motion buttons, and motion status.

For a custom **3D** panel, set display frame **`world`** and **Scene → Mesh up
axis → Z-up**, then reload Lichtblick (Cmd-R) so the mesh loader uses the new
axis. The manufacturer's STL meshes use ROS Z-up; the application's default
Y-up rotates each link mesh independently. Enable these topics:

- `/scene/markers`: SDF tables, fixtures, and obstacles with their colors.
- `/robot/markers`: xArm6 mesh materials and the Ouster sensor bodies.
- `/sensor/lidar/ouster_front/points` and `/sensor/lidar/ouster_rear/points`:
  the two independently decoded Ouster point clouds.
- `/tf` and `/tf_static`: robot and sensor transforms.

Use `/robot/markers` for the robot visuals; leave the additional
`/robot_description` display disabled to avoid overlapping copies.

The bridge serves the robot's mesh assets directly from the container; you
do not need to copy the meshes onto macOS. The host port is bound to loopback.
To use another port, run `FOXGLOVE_PORT=8766 make up` and connect to that port.

For motion, add a **Publish** panel for `/motion/command` with schema
`std_msgs/msg/String` and publish `{"data":"ready"}`, `{"data":"sweep"}`, or
`{"data":"home"}`. `{"data":"stop"}` requests cancellation. Watch
`/motion/status` in a Raw Messages panel for planning, execution, and results.
Each command goes through MoveIt and its collision scene.

```sh
make logs       # startup progress and ROS logs; Ctrl-C leaves the service running
make shell      # a ROS-sourced shell in the running container
make down       # stop the demo; keep build artifacts
```

## Other launch modes

```sh
make mock       # MoveIt + mock hardware + bridge, without Gazebo or sensor data
make headless   # same headless stack as make up
make demo       # use the original primitive arm
make up         # default headless xArm6 workcell with two Ousters
make obstacles  # use the obstacle-course world
make perception # also integrate both point clouds into MoveIt's occupancy map
```

These commands update the same Compose service, replacing the previous mode.
To select the primitive robot for any mode or test, set `ROBOT_MODEL=demo`:

```sh
ROBOT_MODEL=demo make mock
ROBOT_MODEL=demo make smoke-mock
```

On an Intel Linux machine, use `DOCKER_PLATFORM=linux/amd64 make image up`.
The Docker base tag is distribution-specific but follows upstream updates;
set `ROS_IMAGE` to an image digest if you need an immutable base.

## Motion test worlds

`workcell` is the default: a metal workbench, rear screen, fixture, and two
target parts, with the xArm6 mounted on the bench. `obstacle_course` adds a
blocking post and guard wall. `empty` remains available for isolated robot
checks:

```sh
TEST_WORLD=workcell make up
TEST_WORLD=obstacle_course make up
TEST_WORLD=empty make up
make smoke-obstacles
```

The standalone SDF files live in `src/arm_bringup/worlds`. A `robot_mount` SDF
frame specifies the robot base position in both Gazebo and TF. The startup
scene loader reads each SDF collision's dimensions and model/link/collision
poses and applies those same obstacles and colors to MoveIt. Lichtblick's
world markers come from the same SDF source. Mock mode uses the same collision
scene.

The adapter supports static boxes, cylinders, spheres, and planes. It rejects
unsupported mesh/heightmap collisions, nested/includes, dynamic bodies, and
external pose references so an unsupported obstacle cannot silently disappear
from planning. It is a startup snapshot: moving Gazebo objects interactively
does not update MoveIt automatically.

For xArm6, `ready`, `sweep`, and `home` are the free test poses. In
`obstacle_course`, `blocked` intentionally intersects the red post and planning
to it must fail. The smoke check verifies loaded collision geometry, executes
the free sequence, confirms the measured joints, and checks rejection of the
blocked goal without executing it.

## Verify planning and execution

The bootstrap preservation tests use disposable local Git repositories:

```sh
python3 -m unittest discover -s scripts -p test_bootstrap.py -v
```

```sh
make smoke-mock
make smoke
make smoke-obstacles
make bridge-check
make registration-check
```

The three `smoke` commands start separate, temporary headless stacks with their own ROS
domain. It checks that both controllers are active and all six joint states
arrive, asks MoveIt to plan **and execute** `ready`, `sweep` (xArm6), and `home`, and checks the
measured joint positions. The Gazebo check also requires an advancing `/clock`.
The test stops its stack when complete. It shares the build volumes with the
main service, so run builds and smoke checks sequentially.

`bridge-check` checks the running service's WebSocket schemas, ROS streams,
and remote mesh assets. `registration-check` transforms each live cloud into
`world` and checks its distances to the actual SDF surfaces, catching rotated
or displaced scans even when topics and TF names look valid. The Gazebo smoke
checks include this registration check when Ousters are enabled.
To test motion publishing through the bridge as well (moves the simulated arm):

```sh
docker compose exec --user ros dev /usr/local/bin/ros-shell.sh \
  python3 scripts/check_bridge.py --pose ready
```

To check an already-running demo (this moves the simulated arm):

```sh
make shell
ros2 run arm_bringup check_stack --robot xarm6 --mode gazebo --world workcell
# Add --ousters to verify live cloud returns and the sensor TF mounts too.
# Choose --robot demo or --mode mock to match the running stack.
```

## Development

```text
src/
  arm_description/     # URDF/xacro, inertias, limits, mock/Gazebo interfaces
  arm_moveit_config/   # SRDF, KDL IK, OMPL, controllers; xArm6 config in config/xarm6/
  arm_bringup/         # launch, SDF worlds, scene synchronization, integration check
  external/xarm_ros2/  # Git submodule; sparse checkout of xarm_description
  external/gz_sensors_ouster/  # cam_wip + ouster_sim_core submodule
  external/ouster-ros/        # matching pinned driver + ouster-sdk submodule
.gitmodules           # upstream URLs and tracking branches; Git records revisions
third_party/          # UFACTORY attribution and license
docker/               # ROS environment setup
scripts/              # bootstrap, build, demo, and smoke-test entry points
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
Both robots use `world` as the fixed frame, planning group `arm`, and position
commands through `/arm_controller/follow_joint_trajectory`. The xArm6 uses
`link_base` as its base, `link_eef` as its tip, and joints `joint1` through
`joint6`; the primitive arm uses `base_link`, `tool0`, and `joint_1` through
`joint_6`.

The xArm6 wrapper is `arm_description/urdf/xarm6.urdf.xacro`. It loads UFACTORY's
model with limited joint ranges and selects either `mock_components/GenericSystem`
or `gz_ros2_control/GazeboSimSystem`. The local MoveIt configuration includes the
manufacturer's arm self-collision exclusions and the `home` and `ready` states.
Velocity and acceleration limits in our MoveIt configuration are conservative
simulation settings. The integration check reads named poses from the selected
SRDF, so it uses the same targets as the desktop motion-command interface.

`scripts/bootstrap.py` initializes only missing submodules and their missing
nested dependencies. Existing branches, commits, and uncommitted changes are
preserved, even when they differ from the workspace's recorded revisions.
It does not fetch updates or run a recursive checkout over existing work.
Builds after initialization work offline.

The UFACTORY submodule tracks Jazzy. Bootstrap configures a sparse checkout
of `xarm_description` if no sparse selection exists; `colcon.meta` supplies
that package's older CMake compatibility setting. Recursive clones also
initialize the manufacturer's C++ hardware SDK, which `build.sh` excludes
from simulation builds.

## Two Ouster viewpoints

The three repositories under `src/external` are registered Git submodules.
`gz_sensors_ouster` tracks `cam_wip` and includes its `ouster_sim_core`
submodule; `ouster-ros` includes the Ouster SDK. The workspace's Git links
record the integrated commits, and `.gitmodules` records their source URLs.

Develop directly in the Ouster checkouts. A normal recursive clone starts
submodules on detached HEADs; create a work branch before making commits:

```sh
git -C src/external/gz_sensors_ouster switch -c my-sensor-change
git -C src/external/gz_sensors_ouster/third_party/ouster_sim_core switch -c my-core-change
make build
```

Existing local branches, including your current `cam_wip` and core `main`,
are left as they are. Commit and push changes in each edited repository
first. For core changes, also commit and push the updated core Git link in
`gz_sensors_ouster`. Finally, commit the updated Git links in this workspace.
Use `git submodule status --recursive` to inspect the recorded checkouts.
Run `git submodule update --init --recursive` deliberately when you want to
restore the commits recorded by the parent repositories; builds do not run
that command against existing work.

`ouster_front_mount` and `ouster_rear_mount` SDF frames define the fixed sensor
poses. Both OS1-64s run in CPU `raycast` mode at 3 Hz, with different topic
namespaces. Native Ouster packets and metadata feed one `ouster_ros/os_cloud`
process per sensor, producing standard `PointCloud2` messages. No rendering
engine, CUDA, or GPU passthrough is needed for raycasting.

The `cam_wip` decoder requires distinct `sensor_frame` and `point_cloud_frame`
names. Clouds use `<name>/lidar_frame`, `sensor_frame` is `<name>/os_sensor`,
and `pub_static_tf` is false because the sensor URDF owns those transforms.
This prevents the decoder from applying the physical housing's additional
180° rotation and 36.18 mm offset to the simulation's identity XYZ lookup.

```sh
LIDAR_HZ=2.0 make up           # lower the scan rate
ENABLE_OUSTERS=false make up  # robot/world only
make perception              # live dual-cloud occupancy integration
```

Occupancy mapping is opt-in. The known static SDF obstacles are always present
in MoveIt. With perception enabled, `sensors_3d.yaml` adds both clouds to a
4 cm octomap with robot self-filtering. Restart the stack to clear accumulated
occupancy after changing world geometry.

This integration is a bare xArm6 for simulation, without a gripper or physical
robot driver. No robot IP address is used. Physical hardware support requires
the manufacturer's driver and a hardware-specific configuration.

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
python3 scripts/bootstrap.py
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
bash scripts/build.sh
source install/setup.bash
ros2 launch arm_bringup demo.launch.py
# Options: robot:=demo / mode:=mock / world:=obstacle_course / ousters:=false
```

Native host ROS nodes are not automatically connected to ROS nodes inside
Docker Desktop. Use `make shell` for CLI tools; all demo nodes use Cyclone DDS,
domain `42`, and localhost discovery inside the same container.

## Troubleshooting

- **Docker connection error:** start Docker Desktop and wait for its engine.
- **Lichtblick cannot connect:** inspect `make logs`, confirm Foxglove Bridge
  is listening, and check `FOXGLOVE_PORT` against the desktop connection URL.
- **Meshes missing:** the 3D panel needs the bridge's asset capability; use
  `make bridge-check` to test mesh retrieval and streamed ROS data.
- **Arm meshes look disconnected:** set **3D → Scene → Mesh up axis → Z-up**
  and reload Lichtblick. Keep only one robot visual display enabled.
- **Missing transforms after switching layouts:** reload Lichtblick (Cmd-R)
  to resubscribe to the latched `/tf_static` transforms.
- **Slow simulation:** reduce `LIDAR_HZ` or disable Ousters temporarily. Use
  `CMAKE_BUILD_PARALLEL_LEVEL=1` for builds on this 16 GB Mac.
- **No simulated motion:** Gazebo must be playing; the launch starts it with
  `-r`. Inspect `ros2 control list_controllers` in `make shell`.
- **Clean rebuild:** `make down`, then `docker compose down -v` removes the
  generated Docker build/install/log volumes. Your source files remain.

## Upstream references

- [ROS Docker images and supported architectures](https://hub.docker.com/_/ros)
- [Gazebo/ROS version pairings](https://gazebosim.org/docs/latest/ros_installation/)
- [MoveIt 2 binary installation](https://moveit.ai/install-moveit2/binary/)
- [Lyrical gz_ros2_control integration](https://control.ros.org/lyrical/doc/gz_ros2_control/doc/index.html)
- [UFACTORY xArm ROS 2 repository](https://github.com/xArm-Developer/xarm_ros2)
- [Pinned xArm model revision](https://github.com/xArm-Developer/xarm_ros2/tree/d837f2e17f7f36043103b367fca3af8172626245)
- [Ouster simulation cam_wip branch](https://github.com/jcfurey/gz_sensors_ouster/tree/cam_wip)
- [Lichtblick live data connections](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/live-data/)
- [Foxglove ROS bridge](https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge)
